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
    _emg_switch = 0;
    _mode_switch = 0;
}

void RemoteManager::update()
{
    if(_emg_switch){_mode_switch = 2;}
    {
        switch(_mode_switch)
        {
            case 0:
                Drive(_cmd_vel_ros);
                break;
            case 1:
                Drive(_cmd_vel_remote);
                break;
            case 2:
                Stop();
                break;
            default:
                return;
        }
    }
    _cmd_vel_pub.publish(_cmd_vel);
}

void RemoteManager::Drive(const geometry_msgs::Twist& cmd_vel_used)
{
    _cmd_vel = cmd_vel_used;
}

void RemoteManager::Stop()
{
    _cmd_vel = cmd_vel_zero;
}

// void RemoteManager::Accelerate(const geometry_msgs::Twist& cmd_vel_target, double max_linear_accelerate, double max_angular_accelerate)
// {
//     unsigned long time_now = micros();
//     double _dt = (time_now - _time_last);
//     _time_last = time_now;

//     _linear_velocity += constrain((linear_velocity - _linear_velocity)*1e6/_dt, -max_linear_accelerate, max_linear_accelerate) * _dt / 1e6;
//     _angular_velocity += constrain((angular_velocity - _angular_velocity)*1e6/_dt, -max_angular_accelerate, max_angular_accelerate) * _dt / 1e6;

//     _linear_velocity = constrain(_linear_velocity, -1.0, 1.0);
//     _angular_velocity = constrain(_angular_velocity, -1.0, 1.0);

//     Drive(_linear_velocity, _angular_velocity);
// }

// void RemoteManager::DecelerateStop()
// {
//     Accelerate(cmd_vel_zero, 800.0, 120.0);
// }

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
    _emg_switch = emg_msg.data;
}

void RemoteManager::mode_switch_cb(const std_msgs::Int32& mode_msg)
{
    _mode_switch = mode_msg.data;
}
