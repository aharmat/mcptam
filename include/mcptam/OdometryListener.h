#ifndef MCPTAM_ODOMSUBSCRIBER_H
#define MCPTAM_ODOMSUBSCRIBER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>

#include <TooN/TooN.h>
class OdometryListener
{

    public:
        ros::NodeHandle *node;
        std::string odom_topic;

        TooN::Vector<6> current_twist; //track last twist
        ros::Subscriber odometry_subscriber;

        void odomCallback(const geometry_msgs::TwistWithCovarianceStamped &msg);
        OdometryListener(std::string odom_topic, ros::NodeHandle *node);
        int subscribeToOdom();
};
#endif
