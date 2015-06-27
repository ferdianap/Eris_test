#!/usr/bin/python
import rospy
from std_srvs.srv import Empty, EmptyResponse

class Server:
    def __init__(self):
        self.server = rospy.Service("ferdian_example_service",
            Empty,self.callback)
    
    def callback(self,req):
        rospy.loginfo("Got the message")
        rospy.sleep(2)# do something
        return EmptyResponse()
        
rospy.init_node("server")
s = Server()
rospy.loginfo("Server is ready")
rospy.spin()
