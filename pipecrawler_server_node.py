#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Description:
Action Server that controls a pipe crawler's pneumatics
through raspberry pi pins, relay, and solenoid valves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Test with: 

ros2 action send_goal --feedback crawler_action pipecrawler/action/Crawl '{command: {pattern: [2,5,3,6,1,4], timing: [,,,,,,,], looping: True}}'
ros2 action send_goal --feedback crawler_action pipecrawler/action/Crawl '{command: {pattern: [0,0,0,0], looping: False}}'

"""
import threading
from argparse import Action
from os import pipe
import time
import rclpy
from lgs_interfaces.action import Crawler
from lgs_interfaces.msg import Crawlerstate  
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from gpiozero import LED as gpio

d = 2.95
gf = gpio(5)
e = gpio(6)
gb = gpio(16)

def none():
    print("none")

gripper_commands = {
    1:gf.on, #GREEN 
    2:gf.off, #GREEN
    3:e.on, #BLUE 
    4:e.off, #BLUE
    5:gb.on, #RED 
    6:gb.off, #RED
    0:none
}

gripper_command_messages ={
    1:"front_grip_on", #GREEN 
    2:"front_grip_off", #GREEN
    3:"extender_on", #BLUE 
    4:"extender_off", #BLUE
    5:"back_grip_on", #RED 
    6:"back_grip_off", #RED
    0:"none" 
}

class CrawlServer(Node):
    def __init__(self):
        super().__init__('pipecrawler_server')  # Node instance name ()must be matched)
        self.get_logger().info('Initializing Pipecrawler Server')
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._module_state = "back_grip_off"
        self._state = [0,0,0]
        self._state_publisher = self.create_publisher(Crawlerstate, 'crawler_state', 10)
        self._module_publisher = self.create_publisher(String, 'modules_state', 10)
        self._action_server = ActionServer(
            self,                               # Node
            Crawler,                        # Action Type, imported
            'activate_crawler',               # Action Name (Must be matched in other nodes or commands)
            execute_callback = self.execute_callback,        # Callback Function to execute
            goal_callback = self.goal_callback,
            handle_accepted_callback = self.handle_accepted_callback,
            cancel_callback =self.cancel_callback,
            callback_group = ReentrantCallbackGroup())
        
    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def UpdateState(self, command):
        if command != 0:
            self._state[((command+1)//2)-1] = command % 2
        
    def Execute(self, command_list, timing_list):
        for i in range(len(command_list)):
            gripper_commands[command_list[i]]()
            self.UpdateState(command_list[i])
            state_msg = Crawlerstate()
            module_msg = String()
            state_msg.state = self._state
            module_msg.data = gripper_command_messages[command_list[i]]
            self._state_publisher.publish(state_msg)
            self._module_publisher.publish(module_msg)
            self.get_logger().info("waiting, %f seconds" % timing_list[i])
            time.sleep(timing_list[i])

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        _pattern = goal_handle.request.command.pattern
        _timing = goal_handle.request.command.timing

        if goal_handle.request.command.looping:
            self.get_logger().info("Executing looping crawl command")
            while goal_handle.is_active:
                
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                
                self.Execute(_pattern,_timing)
                feedback_ = Crawler.Feedback()
                feedback_.current.state = self._state
                goal_handle.publish_feedback(feedback_)
                              
        else: 
            self.get_logger().info("Executing single command")
            self.Execute(_pattern,_timing)
            feedback_ = Crawler.Feedback()
            goal_handle.publish_feedback(feedback_)
            self.get_logger().info("Goal Succeeded")
            goal_handle.succeed()
        
        result_ = Crawler.Result()
        result_.final.state = self._state
        result_.success = True
        return result_

def main(args=None):
    rclpy.init(args=args)
   
    try: 
        # Create the Action Server Node instance
        pipecrawler_server_instance = CrawlServer()
        
        # Set up mulithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(pipecrawler_server_instance)
        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            pipecrawler_server_instance.destroy_node()
    finally:
        # Shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 