"""ROS-independent IK seeds and ranking of already checked joint candidates.

Seeds are numerical initial guesses, never IK solutions or motion permission.
The caller must check each returned IK solution for collision, FK residual and
current session authority before ranking it here.  A finite collection is not
an exhaustive enumeration of the arm's IK solutions.
"""
import math

import numpy as np


def _joints(value, name):
    try:
        result = np.asarray(value, dtype=float).copy()
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain six finite joint angles') from exc
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain six finite joint angles')
    return result


def _bounds(lower, upper, margin):
    lower, upper = _joints(lower, 'lower'), _joints(upper, 'upper')
    if (isinstance(margin, (bool, np.bool_)) or
            not isinstance(margin, (int, float, np.integer, np.floating)) or
            not math.isfinite(margin) or margin < 0.):
        raise ValueError('margin must be finite and nonnegative')
    with np.errstate(over='ignore'):
        span = upper-lower
    if (not np.all(np.isfinite(span)) or np.any(lower >= upper) or
            np.any(lower + margin >= upper - margin)):
        raise ValueError('Joint bounds must leave a nonempty interval after margin')
    return lower, upper, lower + margin, upper - margin


def _periodic_seed(value, lower, upper, reference):
    """Choose a bounded equivalent angle when one exists; otherwise clamp.

    This only constructs a numerical seed. Candidate continuity scores below
    deliberately use unwrapped travel: bounded joints cannot cross a hard stop
    merely because two endpoint angles have the same trigonometric value.
    """
    first = math.ceil((lower-value)/(2.*math.pi))
    last = math.floor((upper-value)/(2.*math.pi))
    if first <= last:
        near = int(round((reference-value)/(2.*math.pi)))
        turn = min(last, max(first, near))
        return value + turn*(2.*math.pi)
    return float(np.clip(value, lower, upper))


def make_ik_seeds(reference, lower, upper, *, previous=None, ready=None,
                  margin=0., max_seeds=6):
    """Return legal, distinct reference/previous/ready/middle/wrist guesses.

    Optional malformed previous/ready states are ignored. All finite guesses
    are clamped into the requested bound margin, without changing their input.
    Wrist flips use the common roll/pitch/roll seed transform only; no claim is
    made that it preserves this robot's FK. Official IK must solve it again.
    """
    reference = _joints(reference, 'reference')
    lower, upper, seed_lower, seed_upper = _bounds(lower, upper, margin)
    if (isinstance(max_seeds, (bool, np.bool_)) or
            not isinstance(max_seeds, (int, np.integer)) or max_seeds < 1):
        raise ValueError('max_seeds must be a positive integer')
    seeds = []

    def add(value):
        try:
            seed = np.clip(_joints(value, 'seed'), seed_lower, seed_upper)
        except ValueError:
            return
        if not any(np.allclose(seed, old, rtol=0., atol=1e-10) for old in seeds):
            seeds.append(seed)

    add(reference)
    if previous is not None:
        add(previous)
    if ready is not None:
        add(ready)
    add((lower+upper)*.5)
    for direction in (1., -1.):
        flipped = np.clip(reference, seed_lower, seed_upper)
        flipped[4] *= -1.
        for index in (3, 5):
            flipped[index] = _periodic_seed(flipped[index] + direction*math.pi,
                seed_lower[index], seed_upper[index], reference[index])
        add(flipped)
    return [seed.copy() for seed in seeds[:max_seeds]]


def candidate_metrics(q, reference, lower, upper, *, margin=0.):
    """Rank continuity first, with a small reward for distance from hard stops.

    Each displacement and margin is normalized by that joint's full span.
    score = RMS travel + 0.5 * maximum travel - 0.15 * minimum margin.
    Smaller is better; the margin reward is bounded by 0.075, so a tiny gain in
    margin cannot justify a large wrist/arm branch jump. No angle wrapping is
    used. This is a preference among supplied checked solutions, not a plan.
    """
    q, reference = _joints(q, 'candidate'), _joints(reference, 'reference')
    lower, upper, allowed_lower, allowed_upper = _bounds(lower, upper, margin)
    if np.any(q < allowed_lower) or np.any(q > allowed_upper):
        raise ValueError('IK candidate exceeds joint bounds or margin')
    span = upper-lower
    travel = np.abs(q-reference)/span
    rms, maximum = math.hypot(*travel)/math.sqrt(6.), float(np.max(travel))
    if not math.isfinite(rms) or not math.isfinite(maximum):
        raise ValueError('IK continuity arithmetic must stay finite')
    room = float(np.min(np.minimum(q-lower, upper-q)/span))
    score = rms + .5*maximum - .15*room
    return dict(score=score, continuity_rms=rms, continuity_max=maximum,
                normalized_margin=room)


def candidate_score(q, reference, lower, upper, *, margin=0.):
    return candidate_metrics(q, reference, lower, upper, margin=margin)['score']


def select_ik_candidate(candidates, reference, lower, upper, *, margin=0.):
    """Return a copy of the lowest-scoring legal candidate; ties keep order.

    Invalid candidates are excluded, never clamped into purported solutions.
    Bounds/reference errors are caller errors and are rejected even for an
    empty collection. No candidate means no permission to move.
    """
    reference = _joints(reference, 'reference')
    _bounds(lower, upper, margin)
    best, best_score = None, math.inf
    for candidate in candidates:
        try:
            q = _joints(candidate, 'candidate')
            score = candidate_score(q, reference, lower, upper, margin=margin)
        except ValueError:
            continue
        if score < best_score:
            best, best_score = q, score
    if best is None:
        raise ValueError('No legal IK candidate was supplied')
    return best.copy()
