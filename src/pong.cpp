#include <ros/ros.h>
#include <ros/topic.h>

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

// Service includes
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>

#include <math.h>


void colorSensorCallback(const turtlesim::ColorConstPtr& color)
{
    ROS_INFO_THROTTLE(0.1, "Color received (r,g,b) = (%i,%i,%i)", color->r, color->g, color->b);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pong");
    ros::NodeHandle nh;

    ROS_INFO("Reset and remove default turtle1 from the game");
    std_srvs::Empty empty;
    ros::service::call<std_srvs::Empty>("/reset", empty);
    turtlesim::Kill kill;
    kill.request.name = "turtle1";
    ros::service::call<turtlesim::Kill>("/kill", kill);
    

    ROS_INFO("Spawn left turtle");
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle_left";
    spawn.request.x = 1.0;
    spawn.request.y = 5.0;
    spawn.request.theta = M_PI_2;
    ros::service::call<turtlesim::Spawn>("/spawn", spawn);


    ROS_INFO("Spawn right turtle");
    spawn.request.name = "turtle_right";
    spawn.request.x = 10.0;
    spawn.request.y = 5.0;
    spawn.request.theta = M_PI_2;
    ros::service::call<turtlesim::Spawn>("/spawn", spawn);


    ros::Subscriber ball_color_sub = nh.subscribe<turtlesim::Color>("/ball/color_sensor", 10, colorSensorCallback);
    
    //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    //geometry_msgs::Twist twist;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        //twist.linear.x = 0.5;
        //twist.angular.z = 0.0;
        //pub.publish(twist);


        loop_rate.sleep();

        ros::spinOnce();
    }
    
}