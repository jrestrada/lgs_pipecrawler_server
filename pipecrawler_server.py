#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Description: :)
Action Server that controls a pipe crawler's pneumatics
through raspberry pi pins, relay, and solenoid valves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Test with: 

ros2 action send_goal --feedback crawl_through_pipe pipecrawler/action/Crawlaction '{crawlercommand: {crawlpattern: [2,5,3,6,1,4], continuous: True}}'
ros2 action send_goal --feedback crawl_through_pipe pipecrawler/action/Crawlaction '{crawlercommand: {crawlpattern: [0,0,0,0,0,0], continuous: False}}'

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Subscription Topics:
    pipecrawler/Crawlpattern
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""
import threading
from argparse import Action
from os import pipe
import time
import rclpy
from pipecrawler.action import Crawlaction
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int16
# from gpiozero import LED as gpio

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

d = 1
# d = 2.25
# gf = gpio(18)
# e = gpio(17)
# gb = gpio(22)


gripper_commands_consolever = {
    1:gb__on,
    2:gb__off,
    3:e__on,
    4:e__off,
    5:gf__on,
    6:gf__off,
    0:none
}

# gripper_commands = {
#     1:gb.on,
#     2:gb.off,
#     3:e.on,
#     4:e.off,
#     5:gf.on,
#     6:gf.off,
#     0:none
# }

def executeCommands(command_list):
    for i in range(len(command_list)):
            gripper_commands_consolever[command_list[i]]()
            time.sleep(d)

class CrawlactionServer(Node):
    def __init__(self):
        super().__init__('pipecrawler_server')  # Node instance name ()must be matched)

        self._crawl_counter = 0
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._action_server = ActionServer(
            self,                               # Node
            Crawlaction,                        # Action Type, imported
            'crawl_through_pipe',               # Action Name (Must be matched in other nodes or commands)
            execute_callback =self.execute_callback,        # Callback Function to execute
            goal_callback = self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        current_commands_list = goal_handle.request.crawlercommand.crawlpattern

        if goal_handle.request.crawlercommand.continuous:
            self.get_logger().info("Executing continuous crawl command")
            while goal_handle.is_active:
                
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                
                executeCommands(current_commands_list)
                self._crawl_counter += 1
                feedback_msg = Crawlaction.Feedback()
                feedback_msg.count = self._crawl_counter
                goal_handle.publish_feedback(feedback_msg)
                              
        else: 
            self.get_logger().info("Executing single command")
            executeCommands(current_commands_list)
            self._crawl_counter +=1
            feedback_msg = Crawlaction.Feedback()
            feedback_msg.count = self._crawl_counter
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Goal Succeeded")
            goal_handle.succeed()
        
        resultant = Crawlaction.Result()
        resultant.successfulcrawl = True
        return resultant

def main(args=None):
    rclpy.init(args=args)
   
    try: 
        # Create the Action Server Node instance
        pipecrawler_server_instance = CrawlactionServer()
        
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
 