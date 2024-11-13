#include "remote_manager/remote_manager.h"

RemoteManager::RemoteManager(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string cmd_vel_ros, cmd_vel_remote, emg_switch, mode_switch, cmd_vel;
    
    pn.param<std::string>("cmd_vel_ros", cmd_vel_ros, "/naviton/mobile_base_controller/cmd_vel/ros");
    pn.param<std::string>("cmd_vel_remote", cmd_vel_remote, "/mqtt/remote_controller/cmd_vel");
    pn.param<std::string>("emg_switch", emg_switch, "/mqtt/remote_controller/emg_switch");
    pn.param<std::string>("mode_switch", mode_switch, "/mqtt/remote_controller/mode_switch");
    pn.param<std::string>("cmd_vel", cmd_vel, "/naviton/mobile_base_controller/cmd_vel");

    _cmd_vel_ros_sub = nh.subscribe(cmd_vel_ros, 10, &RemoteManager::cmd_vel_ros_cb, this);
    _cmd_vel_remote_sub = nh.subscribe(cmd_vel_remote, 10, &RemoteManager::cmd_vel_remote_cb, this);
    _emg_switch_sub = nh.subscribe(emg_switch, 10, &RemoteManager::emg_switch_cb, this);
    _mode_switch_sub = nh.subscribe(mode_switch, 10, &RemoteManager::mode_switch_cb, this);
    _cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel, 10);
}

void RemoteManager::init()
{
    _emg_switch = false;
    _mode_switch = 0;
}

void RemoteManager::update()
{
    if(_emg_switch){_mode_switch = 2;}
    {
        switch(_mode_switch)
        {
            case 0:
                _cmd_vel = _cmd_vel_ros;
                break;
            case 1:
                _cmd_vel = _cmd_vel_remote;
                break;
            case 2:
                _cmd_vel.linear.x = 0.0;
                _cmd_vel.angular.z = 0.0;
                break;
            default:
                return;
        }
    }
    _cmd_vel_pub.publish(_cmd_vel);
}

void RemoteManager::cmd_vel_ros_cb(const geometry_msgs::Twist& ros_msg)
{
    _cmd_vel_ros = ros_msg;
}

void RemoteManager::cmd_vel_remote_cb(const geometry_msgs::Twist& remote_msg)
{
    _cmd_vel_remote = remote_msg;
}

void RemoteManager::emg_switch_cb(const std_msgs::Bool& emg_msg)
{
    _emg_switch = emg_msg;
}

void RemoteManager::mode_switch_cb(const std_msgs::Int32& mode_msg)
{
    _mode_switch = mode_msg;
}
