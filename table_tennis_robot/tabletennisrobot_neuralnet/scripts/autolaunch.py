#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import roslaunch
from std_srvs.srv import SetBool, SetBoolResponse

gazebo_status = False
is_call = False

def gazebo_switch_callback(req):
    success = True
    message = 'success switch'
    global gazebo_status
    global is_call
    if(req.data == True):
        if(gazebo_status == True):
            success = False
            message = 'service fail. gazebo already open'
            return SetBoolResponse(success,message )
        gazebo_status = True
    elif(req.data == False):
        if(gazebo_status == False):
            success = False
            message = 'service fail. gazebo already close'
            return SetBoolResponse(success,message )
        gazebo_status = False
    is_call = True
    return SetBoolResponse(success,message )

if __name__ == '__main__':
    rospy.init_node('en_Mapping', anonymous=True)
    service = rospy.Service('gazebo_switch', SetBool, gazebo_switch_callback)

    launch = None
    rospy.loginfo("started gazebo switch service") 
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        """
        if(gazebo_status == True):
            rospy.loginfo("gazebo status: true")
        else:
            rospy.loginfo("gazebo status: false")
        """

        if(gazebo_status == True and is_call):
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/denn1slu/ros_workspace/vscode_ws/src/table_tennis_robot/tabletennisrobot_gazebo/launch/ttbot_5f.launch"])
            launch.start()
        elif (gazebo_status == False and is_call):
            launch.shutdown()
            launch = None
        is_call = False
        rate.sleep()