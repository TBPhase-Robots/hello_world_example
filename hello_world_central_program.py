import queue
from bs4 import Tag
import rclpy

from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String, Int32

import numpy as np

class forward_or_back():
    def __init__(self,args=None):

        self.robot_subs = {}
        self.robot_pos = {}
        self.robot_pubs = {}
        self.IDs = []

        rclpy.init(args=args)

        self.node = rclpy.create_node("hello_world")
        rate = self.node.create_rate(2)

        self.robot_ID = self.node.create_subscription("/global/robots/added",Int32,callback=self.get_IDs)

        while rclpy.ok():
            if len(self.IDs) == 0:
                print("No ids recieved")

            elif len(self.robot_pos) == 0: # waits until at least a singular bit of data has been recieved has been idenfied
                print("No robots")

            else:
                self.robot_control()

 
    def get_IDs(self,msg):
        self.IDs.append(msg.data)
        for one_ID in self.IDs:
            try :
                self.robot_subs[one_ID] # checks all IDs are already in subscriber dictionary
            except KeyError: # if they are not it will create a new subscriber
                self.create_subscriber(one_ID)

    def create_subscriber(self,ID):
        tag = "/robot" + str(ID) + "/pose"
        self.robot_subs[ID] = self.node.create_subscription(tag,Pose,callback=self.position_callback,callback_args=ID)

    def create_publisher(self,ID):
        tag = "/robot" + str(ID) + "/vectors"
        self.robot_pubs[ID] = self.node.create_publisher(tag, Vector3, queue_size=10)

    def position_callback(self,pos,ID):
        x = pos.position.x
        y = pos.position.y
        theta = pos.orientation.z

        self.robot_pos[ID] = [x,y,theta]

        print(self.robot_pos)

    def robot_control(self):
    # For the moment, the code will just check where the robot is and then say back or forward depending on what half it is in
        for robot in self.IDs: # get each robots ID number

            message = Vector3()
            x_pos = self.robot_pos[robot][0]
            y_pos = self.robot_pos[robot][1] # gets the robots x and y position
            theta_pos = self.robot_pos[robot][2]

            # this is where to add that robots command
            eg_command = [-x_pos,-y_pos]
            vector = eg_command

            # coordinates must be transformed into the robots frame: use rotation matrix

            theta = theta_pos

            rot_mat = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])

            new_vec = np.matmul(rot_mat,vector) # this should rotate in

            message.x = new_vec[0]
            message.y = new_vec[1]

            print(new_vec)

            try:
                self.robot_pubs[robot].publish(message)
            except KeyError:
                self.create_publisher(robot)
                self.robot_pubs[robot].publish(message)

    def shutdown(self):
        message = Vector3()
        message.x, message.y = 0,0
        self.robot_pub.publish(message)

if __name__ == "__main__":
    go = forward_or_back()
