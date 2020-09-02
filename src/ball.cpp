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
    void penOff(bool off);
    void move();
    void reset();
    void setPoseAbs(double x, double y, double theta);
    void setVel(double x, double theta);
    void randomDirection();

private:
    ros::NodeHandle& nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient teleport_abs_client_;

    turtlesim::PoseConstPtr pose_;

    eDir direction_;
};


cBall::cBall(ros::NodeHandle &nh)
    : nh_(nh)
{
    ROS_INFO("Initialize Turtle Ball");

    reset();

    pose_sub_ = nh_.subscribe<turtlesim::Pose>("/ball/pose", 1, &cBall::poseCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ball/cmd_vel", 1);
    teleport_abs_client_ = nh_.serviceClient<turtlesim::TeleportAbsolute>("/ball/teleport_absolute");

    direction_ = STOP;
};

void cBall::penOff(bool off)
{
    turtlesim::SetPen set_pen;
    set_pen.request.off = off;
    ros::service::call("/ball/set_pen", set_pen);
}

void cBall::move()
{

    if (pose_ == 0)
    {
        ROS_INFO("Pose not yet received");
        setPoseAbs(3.0, 3.0, -0.5);
        
        direction_ = DOWN_RIGHT;
        return;
    }    

    setVel(2.0, 0.0);

    // get current pose
    double x = pose_->x;
    double y = pose_->y;
    double theta = pose_->theta;
/*
    switch (direction_)
    {
    case STOP:
        break; 
    case LEFT:
        setPoseAbs(pose_->x, pose_->y, M_PI);
        setVel(1.0, 0.0);
        break; 
    case UP_LEFT:
        setPoseAbs(pose_->x, pose_->y, 135.0 * M_PI / 180.0);
        setVel(1.0, 0.0);
        break;
    case DOWN_LEFT:
        setPoseAbs(pose_->x, pose_->y, 225.0 * M_PI / 180.0);
        setVel(1.0, 0.0);
        break;
    case RIGHT:
        setPoseAbs(pose_->x, pose_->y, 0.0);
        setVel(1.0, 0.0);
        break;
    case UP_RIGHT:
        setPoseAbs(pose_->x, pose_->y, 45.0 * M_PI / 180.0);
        setVel(1.0, 0.0);
        break;
    case DOWN_RIGHT:
        setPoseAbs(pose_->x, pose_->y, -45.0 * M_PI / 180.0);
        setVel(1.0, 0.0);
        break;
    }
*/
    // Update pose if ball hits the wall
    double new_theta = 0.0;
    if (11.0 < pose_->y) // hit top wall
    {
        if (direction_ == UP_LEFT)
        {
            new_theta = 2.0*M_PI - theta; // (180 - theta) + 180
            setPoseAbs(x, y, new_theta);
            direction_ = DOWN_LEFT;
        }
        if (direction_ == UP_RIGHT)
        {
            new_theta = 2.0*M_PI-theta; // -theta
            setPoseAbs(x, y, new_theta);
            direction_ = DOWN_RIGHT;
        }
    }

    if (1e-3 > pose_->y) // hit bottom wall
    {
        if (direction_ == DOWN_LEFT)
        {
            new_theta = -theta; // (180 - theta) + 180
            setPoseAbs(x, y, new_theta);
            direction_ = UP_LEFT;
        }
        if (direction_ == DOWN_RIGHT)
        {
            new_theta = -theta; // -theta
            setPoseAbs(x, y, new_theta);
            direction_ = UP_RIGHT;
        }
    }

    if (11.0 < pose_->x) // hit right wall
    {
        if (direction_ == UP_RIGHT)
        {
            new_theta = M_PI - theta; // (180 - theta) + 180
            setPoseAbs(x, y, new_theta);
            direction_ = UP_LEFT;
        }
        if (direction_ == DOWN_RIGHT)
        {
            new_theta = -M_PI-theta; // -theta
            setPoseAbs(x, y, new_theta);
            direction_ = DOWN_LEFT;
        }
    }
    
    if (1e-3 > pose_->x) // hit left wall
    {
        if (direction_ == UP_LEFT)
        {
            new_theta = M_PI - theta; // 180 - theta
            setPoseAbs(x, y, new_theta);
            direction_ = UP_RIGHT;
        }
        if (direction_ == DOWN_LEFT)
        {
            new_theta = -(M_PI + theta); // -theta
            setPoseAbs(x, y, new_theta);
            direction_ = DOWN_RIGHT;
        }
    }
};


void cBall::poseCallback(const turtlesim::PoseConstPtr& pose)
{
    pose_ = pose;
    ROS_INFO_THROTTLE(1, "x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f, dir: %i", 
            pose_->x, pose_->y, pose_->theta, pose_->linear_velocity, pose_->angular_velocity, direction_);
};

void cBall::reset()
{
    setPoseAbs(3.0, 3.0, 0.0);
    penOff(true);
}

void cBall::setPoseAbs(double x, double y, double theta)
{
    turtlesim::TeleportAbsolute pose_abs;
    pose_abs.request.x = x;
    pose_abs.request.y = y;
    pose_abs.request.theta = theta;
    ROS_INFO("Set Absolute Pose (x, y, theta) = (%f, %f, %f)", x, y, theta);
    teleport_abs_client_.call(pose_abs);
}

void cBall::setVel(double x, double theta)
{
    geometry_msgs::Twist twist;
    twist.linear.x = x;
    twist.angular.z = theta;
    cmd_vel_pub_.publish<geometry_msgs::Twist>(twist);
}


void cBall::randomDirection()
{
    direction_ = (eDir)((rand() % 6) + 1);
}



int main(int argc, char** argv)
 {
    ros::init(argc, argv, "turtle_pong_ball");

    ros::NodeHandle nh;

    turtlesim::Spawn spawn;
    spawn.request.name = "ball";
    spawn.request.x = 2.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 0.0;
    
    ROS_INFO("Spawn ball turtle");
    ros::service::call<turtlesim::Spawn>("/spawn", spawn);
    

    cBall ball(nh);

    ros::Rate loop_rate(100);
    
    while(ros::ok())
    {
        ball.move();
        ros::spinOnce();
        loop_rate.sleep();
        
    }
}