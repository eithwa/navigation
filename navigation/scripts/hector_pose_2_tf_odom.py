#!/usr/bin/env python

import rospy


from geometry_msgs.msg import PoseStamped
from tf.broadcaster import TransformBroadcaster


class Hector_Pose_2_Odom():
    def __init__(self):

        # ros
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_out_pose_cb)
        #self.odomPub = rospy.Publisher("odom", Odometry, queue_size=3)
        self.odomBroadcaster = TransformBroadcaster()

        self.base_frame_id = "base_footprint"
        self.odom_frame_id = "odom"

        rospy.loginfo("Init Hector_Pose_2_TF_Odom Finish!")

    def slam_out_pose_cb(self,req):
        orientation = req.pose.orientation

        self.odomBroadcaster.sendTransform(
            (req.pose.position.x,req.pose.position.y, 0),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
            )


if __name__ == "__main__":
    rospy.init_node('Hector_Pose_2_TF_Odom')
    Hector_Pose_2_Odom()
    rospy.spin()
