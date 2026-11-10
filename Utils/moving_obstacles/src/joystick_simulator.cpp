#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <vector>
#include <Eigen/Dense>

std::vector<double> obs1_odom(3, 0.0);
std::vector<double> obs2_odom(3, 0.0);

void obs1_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    obs1_odom[0] = msg->pose.pose.position.x;
    obs1_odom[1] = msg->pose.pose.position.y;
    obs1_odom[2] = msg->pose.pose.position.z;
    // ROS_ERROR("obs1_odom: (%f, %f, %f)", obs1_odom[0], obs1_odom[1], obs1_odom[2]);
}

void obs2_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    obs2_odom[0] = msg->pose.pose.position.x;
    obs2_odom[1] = msg->pose.pose.position.y;
    obs2_odom[2] = msg->pose.pose.position.z;
    // ROS_ERROR("obs2_odom: (%f, %f, %f)", obs2_odom[0], obs2_odom[1], obs2_odom[2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_simulator_publisher");
    ros::NodeHandle nh;
    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("/joy", 10);
    ros::Subscriber obs1_pos_sub = nh.subscribe<nav_msgs::Odometry>("obs1_pos", 10, obs1_odom_cb);
    ros::Subscriber obs2_pos_sub = nh.subscribe<nav_msgs::Odometry>("obs2_pos", 10, obs2_odom_cb);

    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(6, 0.0);
    joy_msg.buttons.resize(12, 0);

    std::string obs1_state = "right";
    std::string obs2_state = "right";

    joy_msg.axes[1] = 4.0;
    double rate = 0.01;

    while (ros::ok())
    {
        if(obs1_state == "right"){
            if (obs1_odom[0] > 4)
            {               
                joy_msg.axes[1] -= rate;
                if(joy_msg.axes[1] < -4){
                    obs1_state = "left";
                }
            }
        }
        else{
            if (obs1_odom[0] < -4)
            {
                joy_msg.axes[1] += rate;
                if(joy_msg.axes[1] > 4){
                    obs1_state = "right";
                }
            }
        }

        // if (obs1_state == "right")
        // {
        //     joy_msg.axes[1] = 4.0;
        //     if (obs1_odom[0] > 5)
        //     {
        //         obs1_state = "left";
        //     }
        // }
        // else
        // {
        //     joy_msg.axes[1] = -4.0;
        //     if (obs1_odom[0] < -5)
        //     {
        //         obs1_state = "right";
        //     }            
        // }


        // if (obs2_state == "right")
        // {
        //     joy_msg.axes[4] = 1.0;
        //     if (obs2_odom[0] > 4)
        //     {
        //         obs2_state = "left";
        //     }
        // }
        // else
        // {
        //     joy_msg.axes[4] = -1.0;
        //     if (obs2_odom[0] < -4)
        //     {
        //         obs2_state = "right";
        //     }            
        // }

        joy_pub.publish(joy_msg);

        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    return 0;
}