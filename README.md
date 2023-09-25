# lgs_pipecrawler_server
ROS2 Python node containing an action server that activates the Lateral Gamma Scanner's pneumatic modules to produce the pipecrawler's peristaltic motion.

Run using:
```console
$ ros2 run lgs_pipecrawler_server pipecrawler_server_node.py
```
This server expects a custom action defined by the [lgs_interfaces package](https://github.com/jrestrada/lgs_interfaces/)

These actions can be requested from a terminal using the following syntax:

```console
$ ros2 action send_goal --feedback crawler_action pipecrawler/action/Crawl '{command: {pattern: [2,5,3,6,1,4], timing: [,,,,,,,], looping: True}}
```
When an action request is accepted, the node evaluates the values in the received *pattern* array to activate/deactivate the corresponding relays through the RaspberryPi's GPIO pins using the [gpiozero](https://gpiozero.readthedocs.io/en/stable/) library.
This commands will be followed by a standby period determined by a corresponding value in the *timing* array. The commands will be executed either a single time, or ongoingly, as specified by the field *looping*.

An ongoing pattern will be interrupted when a new action is received and accepted.  

Additionally, the node publishes a String message that reflects the engaged/disengaged state of the individual modules.
