import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
mushu = roslaunch.parent.ROSLaunchParent(uuid, ["/home/haier/catkin_ws/src/testapi/launch/test_node.launch"])
mushu.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
mushu.shutdown()
