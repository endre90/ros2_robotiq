import sys
import rclpy
import os
from rclpy.node import Node
#from ros2_robotiq_base import 
from ros2_robotiq_msgs.msg import Robotiq2FCommand
from ros2_robotiq_msgs.msg import Robotiq2FState
from pymodbus.client.sync import ModbusSerialClient
from math import ceil

class Ros2RobotiqModbusRtu:	

   def __init__(self):
      self.client = None
      
   def connectToDevice(self, device):
      """Connection to the client"""
      self.client = ModbusSerialClient(method='rtu',port=device,stopbits=1, bytesize=8, baudrate=115200, timeout=0.2)
      if not self.client.connect():
          print("Unable to connect to %s" % device)
          return False
      return True

   def disconnectFromDevice(self):
      """Close connection"""
      self.client.close()

   def sendCommand(self, data):   
      """Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""
      #make sure data has an even number of elements   
      if(len(data) % 2 == 1):
         data.append(0)

      #Initiate message as an empty list
      message = []

      #Fill message by combining two bytes in one register
      for i in range(0, len(data)//2):
         message.append((data[2*i] << 8) + data[2*i+1])

      #To do!: Implement try/except 
      self.client.write_registers(0x03E8, message, unit=0x0009)

   def getStatus(self, numBytes):
      """Sends a request to read, wait for the response and returns the Gripper status. The method gets the number of bytes to read as an argument"""
      numRegs = int(ceil(numBytes/2.0))

      #To do!: Implement try/except 
      #Get status from the device
	
      response = self.client.read_holding_registers(0x07D0, numRegs, unit=0x0009)
      # print(response)

      #Instantiate output as an empty list
      output = []

      #Fill the output with the bytes in the appropriate order
      for i in range(0, numRegs):
         output.append((response.getRegister(i) & 0xFF00) >> 8)
         output.append( response.getRegister(i) & 0x00FF)
      
      #Output the result
      return output

class Ros2Robotiq2FBase:
   """Base class (communication protocol agnostic) for sending commands and receiving the status of the Robotic 2F gripper"""

   def __init__(self):

      #Initiate output message as an empty list
      self.message = []

      #Note: after the instantiation, a ".client" member must be added to the object

   def verifyCommand(self, command):
      """Function to verify that the value of each variable satisfy its limits."""
    	#Verify that each variable is in its correct range
      command.r_act = max(0, command.r_act)
      command.r_act = min(1, command.r_act)
      command.r_gto = max(0, command.r_gto)
      command.r_gto = min(1, command.r_gto)
      command.r_atr = max(0, command.r_atr)
      command.r_atr = min(1, command.r_atr)
      command.r_pr  = max(0,   command.r_pr)
      command.r_pr  = min(255, command.r_pr) 
      command.r_sp  = max(0,   command.r_sp)
      command.r_sp  = min(255, command.r_sp)
      command.r_fr  = max(0,   command.r_fr)
      command.r_fr  = min(255, command.r_fr) 
   	
   	#Return the modified command
      return command

   def refreshCommand(self, command):
      """Function to update the command which will be sent during the next sendCommand() call."""
    
	   #Limit the value of each variable
      command = self.verifyCommand(command)

      #Initiate command as an empty list
      self.message = []

      #Build the command with each output variable
      #To-Do: add verification that all variables are in their authorized range
      self.message.append(command.r_act + (command.r_gto << 3) + (command.r_atr << 4))
      self.message.append(0)
      self.message.append(0)
      self.message.append(command.r_pr)
      self.message.append(command.r_sp)
      self.message.append(command.r_fr)     

   def sendCommand(self):
      """Send the command to the Gripper."""    
        
      self.client.sendCommand(self.message)

   def getStatus(self):
      """Request the status from the gripper and return it in the Robotiq2FGripper_robot_input msg type."""

      #Acquire status from the Gripper
      status = self.client.getStatus(6);

      #Message to output
      message = Robotiq2FState()

      #Assign the values to their respective variables
      message.g_act = (status[0] >> 0) & 0x01;        
      message.g_gto = (status[0] >> 3) & 0x01;
      message.g_sta = (status[0] >> 4) & 0x03;
      message.g_obj = (status[0] >> 6) & 0x03;
      message.g_flt =  status[2]
      message.g_pr  =  status[3]
      message.g_po  =  status[4]
      message.g_cu  =  status[5]       

      return message

class Ros2Robotiq2FDriver(Node):

   def __init__(self):
      super().__init__("ros2_robotiq_2f_driver")

      self.robotiq_publisher_rate = 0.05

      self.command_msg = Robotiq2FCommand()
      self.state_msg = Robotiq2FState()

      self.gripper = Ros2Robotiq2FBase()
      self.gripper.client = Ros2RobotiqModbusRtu()

      self.gripper.client.connectToDevice('/dev/ttyUSB0')

      self.robotiq_subscriber = self.create_subscription(
         Robotiq2FCommand, 
         "/robotiq_2f_command",
         self.gripper.refreshCommand,
         10)

      self.robotiq_publisher_ = self.create_publisher(
         Robotiq2FState, 
         "/robotiq_2f_state",
         10)

      self.robotiq_publisher_timer = self.create_timer(
         self.robotiq_publisher_rate, 
         self.robotiq_publisher_callback)

   def robotiq_publisher_callback(self):
      self.status = self.gripper.getStatus()
      self.robotiq_publisher_.publish(self.status)
      
      self.gripper.sendCommand()

def main(args=None):

    rclpy.init(args=args)
    ros2_robotiq_2f_driver = Ros2Robotiq2FDriver()
    rclpy.spin(ros2_robotiq_2f_driver)
    ros2_robotiq_2f_driver.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
