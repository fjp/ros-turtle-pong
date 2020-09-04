#include <ros/ros.h>
#include <ros/topic.h>

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

// Service includes
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/SetPen.h>

#include <math.h>


void colorSensorCallback(const turtlesim::ColorConstPtr& color)
{
    //ROS_INFO_THROTTLE(0.1, "Color received (r,g,b) = (%i,%i,%i)", color->r, color->g, color->b);
}


void spawnPlayerTurtle(std::string name, double x, double y, double theta)
{
    ROS_INFO("Spawn %s", name.c_str());
    turtlesim::Spawn spawn;
    spawn.request.name = name.c_str();
    spawn.request.x = x;
    spawn.request.y = y;
    spawn.request.theta = theta;
    ros::service::call<turtlesim::Spawn>("/spawn", spawn);

    turtlesim::SetPen set_pen;
    set_pen.request.off = true;
    std::string service_name = "/" + name + "/set_pen";
    ros::service::call(service_name, set_pen);
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


    spawnPlayerTurtle("turtle_left", 1.0, 5.0, M_PI_2);

    spawnPlayerTurtle("turtle_right", 10.0, 5.0, M_PI_2);


    ros::Subscriber ball_color_sub = nh.subscribe<turtlesim::Color>("/ball/color_sensor", 10, colorSensorCallback);
    
    //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    //geometry_msgs::Twist twist;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {

        loop_rate.sleep();

        ros::spinOnce();
    }
    
}