#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Description:
Action Server that controls a pipe crawler's pneumatics
through raspberry pi pins, relay, and solenoid valves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Test with: 

ros2 action send_goal --feedback crawler_action pipecrawler/action/Crawleraction '{command: {pattern: [2,5,3,6,1,4], timing: [,,,,,,,], looping: True}}'
ros2 action send_goal --feedback crawler_action pipecrawler/action/Crawleraction '{command: {pattern: [0,0,0,0], looping: False}}'

"""
import threading
from argparse import Action
from os import pipe
import time
import rclpy
from pipecrawler.action import Crawleraction
from pipecrawler.msg import Crawlerstate    
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import LED as gpio

def gb__on():
    print("GB on Test")

def gb__off():
    print("GB off test")

def e__on():
    print("e on Test")

def e__off():
    print("e off test")

def gf__on():
    print("GF on Test")

def gf__off():
    print("GF off test")

def none():
    print("none")
    
gripper_commands_consolever = {
    1:gb__on, # GREEN TUBES
    2:gb__off, # GREEN TUBES
    3:e__on, # BLUE TUBES
    4:e__off, # BLUE TUBES
    5:gf__on, # RED TUBES
    6:gf__off, # RED TUBES
    0:none
}

d = 2.95
gf = gpio(5)
e = gpio(6)
gb = gpio(16)

gripper_commands = {
    1:gb.on, #GREEN 
    2:gb.off, #GREEN
    3:e.on, #BLUE 
    4:e.off, #BLUE
    5:gf.on, #RED 
    6:gf.off, #RED
    0:none
}

class CrawleractionServer(Node):
    def __init__(self):
        super().__init__('pipecrawler_server')  # Node instance name ()must be matched)
        self.get_logger().info('Initializing Pipecrawler Server')
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._state = [0,0,0]
        self._action_server = ActionServer(
            self,                               # Node
            Crawleraction,                        # Action Type, imported
            'crawler_action',               # Action Name (Must be matched in other nodes or commands)
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

    def update(self, command):
        self._state[((command+1)//2)-1] = command

    def execute(self, command_list, timing_list):
        for i in range(len(command_list)):
            gripper_commands[command_list[i]]()
            self.update(command_list[i])
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
                
                self.execute(_pattern,_timing)
                feedback_ = Crawleraction.Feedback()
                feedback_.current.state = self._state
                goal_handle.publish_feedback(feedback_)
                              
        else: 
            self.get_logger().info("Executing single command")
            self.execute(_pattern,_timing)
            feedback_ = Crawleraction.Feedback()
            goal_handle.publish_feedback(feedback_)
            self.get_logger().info("Goal Succeeded")
            goal_handle.succeed()
        
        result_ = Crawleraction.Result()
        result_.final.state = self._state
        result_.success = True
        return result_

def main(args=None):
    rclpy.init(args=args)
   
    try: 
        # Create the Action Server Node instance
        pipecrawler_server_instance = CrawleractionServer()
        
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
 