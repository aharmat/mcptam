#include <mcptam/odomSubscriber.h>

void OdometryListener::odomCallback(
    const geometry_msgs::TwistWithCovarianceStamped &msg
)
{
    this->current_twist[0] = msg.twist.twist.linear.x;
    this->current_twist[1] = msg.twist.twist.linear.y;
    this->current_twist[2] = msg.twist.twist.linear.z;
    this->current_twist[3] = msg.twist.twist.angular.x;
    this->current_twist[4] = msg.twist.twist.angular.y;
    this->current_twist[5] = msg.twist.twist.angular.z;

}

OdometryListener::OdometryListener(
    std::string odom_topic,
    ros::NodeHandle *node
)
{
    this->node = node;
    this->odom_topic = odom_topic;
}

int OdometryListener::subscribeToOdom()
{
    odometry_subscriber = this->node->subscribe(
        this->odom_topic,
        50,
        &OdometryListener::odomCallback,
        this
    );
}



