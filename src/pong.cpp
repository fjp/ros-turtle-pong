#include <ros/ros.h>
#include <ros/topic.h>

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    geometry_msgs::Twist twist;

    ros::Rate loop_rate(0.1);

    while (ros::ok())
    {
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        pub.publish(twist);


        loop_rate.sleep();

        //ros::spinOnce();
    }
    
}