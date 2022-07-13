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


        while not rospy.is_shutdown():
            if not hasattr(self,'IDs'):
                rospy.sleep(1)
                print("No robots")
            else:
                pass
 
    def get_IDs(self,msg):
        print("robot detected")
        self.IDs = msg.data.split() # split func creates a list of all the individual words and then converts them to an integers
        new_ID = self.IDs[-1]
        self.create_subscriber(new_ID)

    def create_subscriber(self,ID):
        tag = "/robot_" + str(ID) + "_position"
        self.robot_subs[ID] = rospy.Subscriber(tag,Pose,callback=self.position_callback,callback_args=ID)

    def position_callback(self,pos,ID):
        x = pos.position.x
        y = pos.position.y
        theta = pos.orientation.z

        self.robot_pos[ID] = [x,y,theta]
        print(self.robot_pos)

if __name__ == "__main__":
    go = forward_or_back()
