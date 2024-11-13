#ifndef REMOTE_MANAGER_H
#define REMOTE_MANAGER_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

class Remote_manager
{
    public:
        Remote_manager(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void init();
        void update();
        void cmd_vel_ros_cb();
        void cmd_vel_remote_cb();
        void emg_switch_cb();
        void mode_switch_cb();

    private:
        ros::Subscriber _cmd_vel_ros_sub;
        ros::Subscriber _cmd_vel_remote_sub;
        ros::Subscriber _emg_switch_sub;
        ros::Subscriber _mode_switch_sub;
        ros::Publisher _cmd_vel_pub;

        geometry_msgs::Twist _cmd_vel_ros;
        geometry_msgs::Twist _cmd_vel_remote;
        std_msgs::Bool _emg_switch;
        std_msgs::Int32 _mode_switch;
        geometry_msgs::Twist _cmd_vel;

};

#endif