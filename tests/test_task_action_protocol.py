"""ROS Action protocol fixtures only. Domain 86 required; no robot driver/CAN."""
import os
import threading
import time
import unittest
from types import SimpleNamespace

@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID')=='86','Run isolated ROS protocol checks in domain 86')
class ActionProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.action import ActionServer,CancelResponse
        from moveit_msgs.action import ExecuteTrajectory
        from std_srvs.srv import SetBool
        cls.context=Context();rclpy.init(context=cls.context)
        cls.node=Node('task_protocol_fixture',context=cls.context)
        cls.mode='success';cls.received=0;cls.gates=[];cls.canceled=False
        def execute(handle):
            cls.received+=1
            deadline=time.monotonic()+3
            while cls.mode=='wait' and time.monotonic()<deadline:
                if handle.is_cancel_requested:
                    cls.canceled=True;handle.canceled()
                    result=ExecuteTrajectory.Result();result.error_code.val=-7
                    return result
                time.sleep(.01)
            result=ExecuteTrajectory.Result()
            if cls.mode=='failure':
                handle.abort();result.error_code.val=-4
            else:
                handle.succeed();result.error_code.val=1
            return result
        def gate(request,response):
            cls.gates.append(request.data);response.success=True;return response
        cls.server=ActionServer(cls.node,ExecuteTrajectory,'/execute_trajectory',execute,
                                cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        cls.gate=cls.node.create_service(SetBool,'/control_enable',gate,callback_group=ReentrantCallbackGroup())
        cls.executor=MultiThreadedExecutor(num_threads=3,context=cls.context)
        cls.executor.add_node(cls.node)
        cls.thread=threading.Thread(target=cls.executor.spin,daemon=True);cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.destroy();cls.executor.shutdown(timeout_sec=3);cls.thread.join(timeout=3)
        cls.node.destroy_node();cls.context.try_shutdown()

    def setUp(self):
        from scripts.task.common.moveit_client import MoveItClient
        from scripts.task.common.schema import DEFAULTS
        from scripts.task.common.robot_feedback import Sample
        self.client=MoveItClient(dict(DEFAULTS))
        # Inject a read-only sample; test transport semantics, not physical tracking.
        self.client.stable=lambda *a,**kw:SimpleNamespace(q=(0.,)*6)
        self.client.feedback.health=lambda *_:None
        self.client.current=lambda:Sample((0.,)*6,time.time(),time.monotonic())
        self.addCleanup(self.client.close)
        type(self).gates=[];type(self).received=0;type(self).canceled=False

    def trajectory(self):
        from moveit_msgs.msg import RobotTrajectory
        return RobotTrajectory()

    def test_success_uses_official_execute_action(self):
        type(self).mode='success'
        self.client.execute(self.trajectory(),[0.]*6,.1)
        self.assertEqual(type(self).received,1)
        self.assertEqual(type(self).gates,[])

    def test_failed_result_then_gate_close(self):
        type(self).mode='failure'
        with self.assertRaisesRegex(RuntimeError,'execution failed'):
            self.client.execute(self.trajectory(),[0.]*6,.1)
        result=self.client.stop()
        self.assertEqual(result['gate'],'closed')
        self.assertEqual(type(self).gates,[False])

    def test_cancel_interrupts_wait_and_uses_cancel_protocol(self):
        type(self).mode='wait'
        timer=threading.Timer(.5,self.client.cancel_requested.set)
        timer.start();self.addCleanup(timer.cancel)
        with self.assertRaises(InterruptedError):
            self.client.execute(self.trajectory(),[0.]*6,.1)
        result=self.client.stop()
        self.assertEqual(result['gate'],'closed')
        self.assertTrue(type(self).canceled)
        self.assertEqual(type(self).received,1)

    def test_cancel_then_explicit_recovery_then_new_execution(self):
        type(self).mode='wait'
        timer=threading.Timer(.5,self.client.cancel_requested.set)
        timer.start();self.addCleanup(timer.cancel)
        with self.assertRaises(InterruptedError):
            self.client.execute(self.trajectory(),[0.]*6,.1)
        self.client.stop()
        self.assertTrue(self.client.task_gate_closed)
        self.client.cancel_requested.clear()
        type(self).mode='success'
        self.client.command_feedback.get=lambda _:self.client.current()
        self.client.restore_task_control()
        self.assertFalse(self.client.task_gate_closed)
        self.assertEqual(type(self).gates,[False,False,True])
        self.client.execute(self.trajectory(),[0.]*6,.1)
        self.assertEqual(type(self).received,3)  # canceled task, gated hold, new task

if __name__=='__main__':unittest.main()
