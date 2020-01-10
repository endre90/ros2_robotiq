from ros2_robotiq_msgs.msg import Robotiq2FCommand
from ros2_robotiq_msgs.msg import Robotiq2FState

class Ros2Robotiq2FBase:
"""Base class (communication protocol agnostic) for sending commands and receiving the status of the Robotic 2F gripper"""

    def __init__(self):

        #Initiate output message as an empty list
        self.message = []

        #Note: after the instantiation, a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Function to verify that the value of each variable satisfy its limits."""
    	   	
   	#Verify that each variable is in its correct range
   	command.rACT = max(0, command.rACT)
   	command.rACT = min(1, command.rACT)
   	
   	command.rGTO = max(0, command.rGTO)
   	command.rGTO = min(1, command.rGTO)

   	command.rATR = max(0, command.rATR)
   	command.rATR = min(1, command.rATR)
   	
   	command.rPR  = max(0,   command.rPR)
   	command.rPR  = min(255, command.rPR)   	

   	command.rSP  = max(0,   command.rSP)
   	command.rSP  = min(255, command.rSP)   	

   	command.rFR  = max(0,   command.rFR)
   	command.rFR  = min(255, command.rFR) 
   	
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
        self.message.append(command.rACT + (command.rGTO << 3) + (command.rATR << 4))
        self.message.append(0)
        self.message.append(0)
        self.message.append(command.rPR)
        self.message.append(command.rSP)
        self.message.append(command.rFR)     

    def sendCommand(self):
        """Send the command to the Gripper."""    
        
        self.client.sendCommand(self.message)

    def getStatus(self):
        """Request the status from the gripper and return it in the Robotiq2FGripper_robot_input msg type."""

        #Acquire status from the Gripper
        status = self.client.getStatus(6);

        #Message to output
        message = inputMsg.Robotiq2FGripper_robot_input()

        #Assign the values to their respective variables
        message.gACT = (status[0] >> 0) & 0x01;        
        message.gGTO = (status[0] >> 3) & 0x01;
        message.gSTA = (status[0] >> 4) & 0x03;
        message.gOBJ = (status[0] >> 6) & 0x03;
        message.gFLT =  status[2]
        message.gPR  =  status[3]
        message.gPO  =  status[4]
        message.gCU  =  status[5]       

        return message