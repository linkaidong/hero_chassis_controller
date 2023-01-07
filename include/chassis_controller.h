//
// Created by LKD on 2023/1/7.
//

#ifndef CHASSIS_CONTROLLER_CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_CHASSIS_CONTROLLER_H

#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>                     
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <hardware_interface/joint_command_interface.h> 
#include <memory>

#define Wheel_Radius 0.07625

namespace chassis_controller{

class ChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    ChassisController() = default;
    ~ChassisController() override;

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
    void update(const ros::Time &time, const ros::Duration &period) override;

    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
    //Pid controller
    control_toolbox::Pid front_left_pid_controller, front_right_pid_controller, back_left_pid_controller, back_right_pid_controller;
private:
    //a Counter
    int loop_count_;
    //expected velocity of wheels
    double vel_wheel_expectation[5];
    //actual velocity of wheels
    double vel_wheel_real[5];
    //expected chassis speed set
    double V_x, V_y, V_yaw;
    //the speed of chassis
    double Vx_chassis, Vy_chassis, Vyaw_chassis;
    //vel_wheel_expectation - vel_wheel_real
    double error[5];
    //Odometer calculation
    double dt;
    double displacement_x, displacement_y, theta;
    double Wheel_track;
    double Wheel_base;
    //Chassis coordinate system speed mode and global coordinate system speed mode
    bool ChassisSpeedMode;

    ros::Time last_time;
    ros::Time now_time;

    ros::Publisher pub;
    ros::Subscriber sub;

    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_;

    nav_msgs::Odometry odom;
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;

    tf::StampedTransform transform;
    geometry_msgs::Vector3Stamped stamped_in, stamped_out;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;             //Transfer position information related to coordinate system

    void Transform_broadcast();
    void Odometry_publish();
    void TF_SpeedControl();
    void get_chassis_state(const geometry_msgs::TwistConstPtr &msg);
    void calculation_wheel_vel();
    void calculation_chassis_vel();
};

} //namespace chassis_controller
#endif  //CHASSIS_CONTROLLER_CHASSIS_CONTROLLER_H