#include <ros/ros.h>
#include <ros/subscriber.h>
#include <turtlesim/Pose.h>

// Service includes
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>

// topic type includes
#include <geometry_msgs/Twist.h>

#include <math.h>

#define deg2rad(angle) (angle * M_PI / 180.0)


enum eDir { 
    STOP = 0, 
    LEFT = 1, 
    UP_LEFT = 2,
    DOWN_LEFT = 3,
    RIGHT = 4,
    UP_RIGHT = 5,
    DOWN_RIGHT = 6
};

class cBall
{
public:
    cBall(ros::NodeHandle &nh);

    void poseCallback(const turtlesim::PoseConstPtr& pose);
    void poseLeftCallback(const turtlesim::PoseConstPtr& pose);
    void poseRightCallback(const turtlesim::PoseConstPtr& pose);
    void penOff(bool off);
    void move();
    void reset();
    void setPoseAbs(double x, double y, double theta);
    void setVel(double x, double theta);
    void randomDirection();

    void checkPlayerCollision();
    void updateDirection();
    double randomAngle();

private:
    ros::NodeHandle& nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber pose_left_sub_;
    ros::Subscriber pose_right_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient teleport_abs_client_;

    turtlesim::PoseConstPtr pose_;
    turtlesim::PoseConstPtr pose_left_;
    turtlesim::PoseConstPtr pose_right_;

    eDir direction_;
};