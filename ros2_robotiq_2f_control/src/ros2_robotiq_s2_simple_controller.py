import sys
import rclpy
import os
import time
from rclpy.node import Node
from ros2_robotiq_msgs.msg import Robotiq2FCommand
from ros2_robotiq_msgs.msg import Robotiq2FState
from time import sleep

def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = Robotiq2FCommand();
        command.r_act = 1
        command.r_gto = 1
        command.r_sp  = 255
        command.r_fr  = 150

    if char == 'r':
        command = Robotiq2FCommand();
        command.r_act = 0

    if char == 'c':
        command.r_pr = 255

    if char == 'o':
        command.r_pr = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.r_pr = int(char)
        if command.r_pr > 255:
            command.r_pr = 255
        if command.r_pr < 0:
            command.r_pr = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.r_sp += 25
        if command.r_sp > 255:
            command.r_sp = 255
            
    if char == 'l':
        command.r_sp -= 25
        if command.r_sp < 0:
            command.r_sp = 0

            
    if char == 'i':
        command.r_fr += 25
        if command.r_fr > 255:
            command.r_fr = 255
            
    if char == 'd':
        command.r_fr -= 25
        if command.r_fr < 0:
            command.r_fr = 0

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  r_act = '  + str(command.r_act)
    currentCommand += ', r_gto = '  + str(command.r_gto)
    currentCommand += ', r_atr = '  + str(command.r_atr)
    currentCommand += ', r_pr = '   + str(command.r_pr )
    currentCommand += ', r_sp = '   + str(command.r_sp )
    currentCommand += ', r_fr = '   + str(command.r_fr )


    print(currentCommand)

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    
    strAskForCommand += '-->'

    return input(strAskForCommand)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('ros2_robotiq_2f_simple_controller')

    msg = Robotiq2FCommand()
   
    publisher = node.create_publisher(Robotiq2FCommand, '/robotiq_2f_command', 10)

    while 1:

        msg = genCommand(askForCommand(msg), msg)   
        publisher.publish(msg)

        time.sleep(0.5)

    # timer_period = 0.5
    # timer = node.create_timer(timer_period, timer_callback)
    rclpy.spin(node)
    # node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()