import queue
import rospy2 as rospy

from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String

class forward_or_back():
    def __init__(self):

        self.robot_subs = {}
        self.robot_pos = {}

        rospy.init_node("hello_world")

        self.robot_ID = rospy.Subscriber("/robot_IDs",String,callback=self.get_IDs)
        self.robot_pub = rospy.Publisher("/vectors",Vector3,queue_size=10)

        while not rospy.is_shutdown():
            if not hasattr(self,'IDs'): # waits until at least a singular ID has been idenfied
                rospy.sleep(1)
                print("No robots")
            else:
                rospy.spin()
 
    def get_IDs(self,msg):
        self.IDs = msg.data.split() # split func creates a list of all the individual words and then converts them to an integers
        for one_ID in self.IDs:
            try :
                self.robot_subs[one_ID] # checks all IDs are already in subscriber dictionary
            except KeyError: # if they are not it will create a new subscriber
                self.create_subscriber(one_ID)

    def create_subscriber(self,ID):
        tag = "/robot_" + str(ID) + "_position"
        self.robot_subs[ID] = rospy.Subscriber(tag,Pose,callback=self.position_callback,callback_args=ID)

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
            x_pos = self.robot_pos[robot][0] # gets the robots x position
            if x_pos > 0:
                message.x = -1
            else:
                message.x = 1

            self.robot_pos.publish(message)

if __name__ == "__main__":
    go = forward_or_back()
