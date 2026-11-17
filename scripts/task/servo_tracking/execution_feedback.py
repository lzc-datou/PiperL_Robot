"""Pure observation of downstream Cartesian capability; never sends a command.

Known DLS mapping loss is tied to its exact controller request. Other losses are
qualified only after a stable request, fresh observations of the complete chain
and a stable sustained deficit. This deliberately leaves transient/delayed or
missing data unknown instead of interpreting every tracking error as saturation.
"""
from collections import deque
import math

import numpy as np

from .cartesian_control import ExecutionFeedback


STAGES = ('servo', 'controller', 'measured')
BLOCKS = (slice(0, 3), slice(3, 6))


def _vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError('Execution observation must contain six finite values')
    return result.copy()


def _time(value):
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)) or not math.isfinite(value):
        raise ValueError('Execution observation requires a finite monotonic time')
    return float(value)


class ExecutionObserver:
    """Conservative multi-stage saturation feedback for ``CartesianController``.

    All times are monotonic receipt times; all twists are base-frame TCP twists
    in m/s and rad/s. Call reset with every controller/session reset. Repeated
    telemetry with an unchanged source stamp must not be presented as new data.
    Stable-window qualification covers the configured downstream delay budget;
    a longer unmodelled actuator delay cannot be distinguished from a sustained
    physical capability deficit by passive observations alone.
    """
    def __init__(self, freshness=.10, stable_window=.16, saturation_window=.08,
                 linear_noise=.005, angular_noise=.025, relative_noise=.10):
        for value in (freshness, stable_window, saturation_window, linear_noise,
                      angular_noise, relative_noise):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Observer settings must be finite positive numbers')
        if stable_window < freshness or saturation_window > freshness:
            raise ValueError('Stable window must cover freshness; qualification must fit it')
        self.freshness = float(freshness)
        self.stable_window = float(stable_window)
        self.saturation_window = float(saturation_window)
        self.noise = (float(linear_noise), float(angular_noise))
        self.relative_noise = float(relative_noise)
        self.reset()

    def reset(self):
        self.requests = deque()
        self.observations = {stage: deque() for stage in STAGES}
        self.joints = {}
        self._candidate = None
        self._candidate_since = None
        self.reason = 'no_request'
        self.limited_sources = ()

    def _clear_candidate(self, reason):
        self._candidate = self._candidate_since = None
        self.reason, self.limited_sources = reason, ()

    def record_request(self, now, requested_twist, mapped_twist):
        now, requested, mapped = _time(now), _vector(requested_twist), _vector(mapped_twist)
        if self.requests and now < self.requests[-1][0]:
            raise ValueError('Execution request time moved backwards')
        if self.requests and now == self.requests[-1][0]:
            self.requests.pop()
        self.requests.append((now, requested, mapped))
        horizon = max(.6, self.stable_window+self.saturation_window+self.freshness)
        while self.requests and now-self.requests[0][0] > horizon:
            self.requests.popleft()

    def observe(self, stage, now, twist):
        if stage not in STAGES:
            raise ValueError('Unknown execution stage')
        try:
            now, twist = _time(now), _vector(twist)
        except (ValueError, TypeError, OverflowError):
            self.observations[stage].clear()
            self._clear_candidate(stage+'_invalid')
            return False
        samples = self.observations[stage]
        if samples and now <= samples[-1][0]:
            return False
        samples.append((now, twist))
        while samples and now-samples[0][0] > max(.6, self.stable_window+self.freshness):
            samples.popleft()
        return True

    def observe_joints(self, stage, now, q, model, velocities=None):
        """Observe J(q)qdot, estimating qdot from positions when absent.

        Differencing spans 2–100 ms and uses the midpoint Jacobian. It neither
        clips nor wraps measured angles. Invalid time gaps reset the estimator
        and invalidate this stage until a new velocity can be observed.
        """
        if stage not in STAGES:
            raise ValueError('Unknown execution stage')
        try:
            now, q = _time(now), _vector(q)
            velocity = None if velocities is None else _vector(velocities)
        except (ValueError, TypeError, OverflowError):
            self.observations[stage].clear()
            self.joints.pop(stage, None)
            self._clear_candidate(stage+'_invalid')
            return False
        previous = self.joints.get(stage)
        if previous is not None and now <= previous[0]:
            return False
        if velocities is not None:
            self.joints[stage] = (now, q)
            reference = q
        else:
            if previous is None or now-previous[0] > .10:
                self.joints[stage] = (now, q)
                self.observations[stage].clear()
                self._clear_candidate('velocity_warmup')
                return False
            if now-previous[0] < .002:
                return False
            self.joints[stage] = (now, q)
            velocity, reference = (q-previous[1])/(now-previous[0]), (q+previous[1])*.5
        try:
            jacobian = np.asarray(model.jacobian(reference), dtype=float)
            if jacobian.shape != (6, 6) or not np.all(np.isfinite(jacobian)):
                raise ValueError('invalid Jacobian')
        except (ValueError, TypeError, OverflowError):
            self.observations[stage].clear()
            self._clear_candidate(stage+'_invalid')
            return False
        return self.observe(stage, now, jacobian @ velocity)

    def _threshold(self, block_index, reference):
        return self.noise[block_index] + self.relative_noise*float(np.linalg.norm(reference))

    def _stable(self, values, reference):
        return all(np.linalg.norm(value[block]-reference[block]) <=
                   self._threshold(index, reference[block])*.5
                   for value in values for index, block in enumerate(BLOCKS))

    def _unknown(self, now, reason):
        self._clear_candidate(reason)
        if self.requests:
            command_time, requested, _ = self.requests[-1]
        else:
            command_time, requested = now, np.zeros(6)
        return ExecutionFeedback(now, command_time, requested.copy(), (False, False),
                                 effective_twist=None, valid=False)

    def feedback(self, now):
        now = _time(now)
        if not self.requests:
            return self._unknown(now, 'no_request')
        command_time, requested, mapped = self.requests[-1]
        if not 0 <= now-command_time <= self.freshness:
            return self._unknown(now, 'request_stale')
        # This difference belongs to the exact current request, not to a plant
        # sample delayed by transport, acceleration or encoder filtering.
        mapped_limited = tuple(np.linalg.norm(requested[block]-mapped[block]) >
                               self._threshold(index, requested[block])
                               for index, block in enumerate(BLOCKS))
        if any(mapped_limited):
            self._clear_candidate('dls_limited')
            self.limited_sources = ('dls',)
            effective = requested.copy()
            for is_limited, block in zip(mapped_limited, BLOCKS):
                if is_limited:
                    effective[block] = mapped[block]
            # The unqualified other block has no verified spare capacity.
            # limited+zero residual makes the controller freeze/unwind only;
            # it does not fabricate an additional back-calculation correction.
            return ExecutionFeedback(command_time, command_time, requested.copy(),
                (True, True), effective, source='command', valid=True)
        for stage in STAGES:
            samples = self.observations[stage]
            if not samples or not 0 <= now-samples[-1][0] <= self.freshness:
                return self._unknown(now, stage+'_stale')
        boundary = now-self.stable_window
        preceding = [entry for entry in self.requests if entry[0] <= boundary]
        history = ([preceding[-1]] if preceding else []) + [entry for entry in self.requests if entry[0] > boundary]
        if (not preceding or any(b[0]-a[0] > self.freshness for a, b in zip(history, history[1:])) or
                not self._stable([entry[1] for entry in history], requested) or
                not self._stable([entry[2] for entry in history], mapped)):
            return self._unknown(now, 'request_changing')
        latest = {stage: self.observations[stage][-1][1] for stage in STAGES}
        effective = requested.copy()
        limited, sources = [], []
        for index, block in enumerate(BLOCKS):
            reference = mapped[block]
            norm = float(np.linalg.norm(reference))
            threshold = self._threshold(index, reference)
            candidates = []
            if norm > threshold:
                direction = reference/norm
                for stage in STAGES:
                    value = latest[stage][block]
                    progress = float(np.dot(value, direction))
                    lateral = np.linalg.norm(value-progress*direction)
                    # Opposing/misaligned samples can be a frame mismatch or a
                    # changing robot configuration; do not back-calculate them.
                    if progress < -self.noise[index] or progress > norm+threshold or lateral > threshold:
                        return self._unknown(now, 'execution_direction_changing')
                    if norm-progress > threshold:
                        candidates.append((progress, stage, value))
            elif any(np.linalg.norm(latest[stage][block]) > threshold for stage in STAGES):
                return self._unknown(now, 'execution_direction_changing')
            limited.append(bool(candidates))
            if candidates:
                _, source, value = min(candidates, key=lambda item: item[0])
                effective[block] = value
                sources.append(source)
        # Small telemetry noise may swap which adjacent stage is numerically
        # slowest. The stable capacity flags, rather than that tie, identify a
        # qualification window; per-stage stability is checked below.
        key = tuple(limited)
        if self._candidate != key:
            self._candidate, self._candidate_since = key, now
        if now-self._candidate_since < self.saturation_window:
            self.reason = 'execution_settling'
            return ExecutionFeedback(now, command_time, requested.copy(), (False, False),
                                     effective_twist=None, valid=False)
        for stage in STAGES:
            samples = [sample for sample in self.observations[stage]
                       if self._candidate_since <= sample[0] <= now]
            if len(samples) < 3 or samples[-1][0]-samples[0][0] < self.saturation_window*.8:
                # Slower (e.g. 30Hz) sources may need one further message to
                # span the window; waiting must not restart that same window.
                self.reason = 'execution_settling'
                return ExecutionFeedback(now, command_time, requested.copy(), (False, False),
                                         effective_twist=None, valid=False)
            if not self._stable([sample[1] for sample in samples], latest[stage]):
                return self._unknown(now, 'execution_changing')
        self.reason = 'execution_limited' if any(limited) else 'execution_unlimited'
        self.limited_sources = tuple(sources)
        # All three stages are fresh. For a complete-chain observation prefer
        # measured capability when it is lower than either command-side stage.
        stamp = min(self.observations[stage][-1][0] for stage in STAGES)
        aligned = [entry for entry in self.requests if entry[0] <= stamp]
        if not aligned:
            return self._unknown(now, 'execution_unmatched')
        command_time, aligned_requested, _ = aligned[-1]
        for is_limited, block in zip(limited, BLOCKS):
            if not is_limited:
                effective[block] = aligned_requested[block]
        source = 'measured' if 'measured' in sources else 'command'
        return ExecutionFeedback(stamp, command_time, aligned_requested.copy(), tuple(limited),
                                 effective, source=source, valid=True)
