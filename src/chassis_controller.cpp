//
// Created by LKD on 2023/1/7.
//

#include "chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller {

    ChassisController::~ChassisController() {
        sub.shutdown(); //Unsubscribe the callback associated with this Subscriber
        pub.shutdown(); //Shutdown the advertisement associated with this Publisher
    }

    bool ChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        controller_nh.getParam("Wheel_track",Wheel_track);
        controller_nh.getParam("Wheel_base",Wheel_base);
        controller_nh.getParam("ChassisSpeedMode", ChassisSpeedMode);

        //get joint handle from hardware interface
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        //load PID Controller using gains set on parameter server
        front_left_pid_controller.init(ros::NodeHandle(controller_nh, "front_left_pid"));
        front_right_pid_controller.init(ros::NodeHandle(controller_nh, "front_right_pid"));
        back_left_pid_controller.init(ros::NodeHandle(controller_nh, "back_left_pid"));
        back_right_pid_controller.init(ros::NodeHandle(controller_nh, "back_right_pid"));

        V_x = 0.0;
        V_y = 0.0;
        V_yaw = 0.0;

        //Start realtime state publisher
        controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>>(root_nh, "state", 1);

        sub = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisController::get_chassis_state, this);
        pub = root_nh.advertise<nav_msgs::Odometry>("odom", 50);

        return true;
    }
    
    void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
        now_time = time;

        vel_wheel_real[1] = front_left_joint_.getVelocity();
        vel_wheel_real[2] = front_right_joint_.getVelocity();
        vel_wheel_real[3] = back_left_joint_.getVelocity();
        vel_wheel_real[4] = back_right_joint_.getVelocity();

        //calculate the speed of chassis
        calculation_chassis_vel();
        //broadcast Transform from "base_link" to "odom"
        Transform_broadcast();
        //publish the odometry message over ROS
        Odometry_publish();
        // Speed control in world coordinates using tf calculation
        TF_SpeedControl();
        //calculate the speed of four wheels
        calculation_wheel_vel();

        error[1] = vel_wheel_expectation[1] - vel_wheel_real[1];
        error[2] = vel_wheel_expectation[2] - vel_wheel_real[2];
        error[3] = vel_wheel_expectation[3] - vel_wheel_real[3];
        error[4] = vel_wheel_expectation[4] - vel_wheel_real[4];

        //set command for wheels; pid_controller.computeCommand :returns PID command
        front_left_joint_.setCommand(front_left_pid_controller.computeCommand(error[1], period));
        front_right_joint_.setCommand(front_right_pid_controller.computeCommand(error[2], period));
        back_left_joint_.setCommand(back_left_pid_controller.computeCommand(error[3], period));
        back_right_joint_.setCommand(back_right_pid_controller.computeCommand(error[4], period));

        if(loop_count_ % 10 == 0){
            if(controller_state_publisher_ && controller_state_publisher_->trylock()){
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = vel_wheel_expectation[1];
                controller_state_publisher_->msg_.process_value = vel_wheel_real[1];
                controller_state_publisher_->msg_.error = error[1];
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = front_left_pid_controller.computeCommand(error[1], period);

                double dummy;
                bool antiwindup;
                front_left_pid_controller.getGains(controller_state_publisher_->msg_.p,
                controller_state_publisher_->msg_.i,
                controller_state_publisher_->msg_.d,
                controller_state_publisher_->msg_.i_clamp,
                dummy,
                antiwindup);
                //msg_.antiwindup's type is unsigned char
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;
        last_time = now_time;       
    }


    void ChassisController::get_chassis_state(const geometry_msgs::TwistConstPtr &msg) {
        V_x = msg->linear.x;
        V_y = msg->linear.y;
        V_yaw = msg->angular.z;
    }

    void ChassisController::calculation_wheel_vel() {
        vel_wheel_expectation[1] = ( V_x - V_y - V_yaw * ( Wheel_base + Wheel_track )/2 )/ Wheel_Radius;
        vel_wheel_expectation[2] = ( V_x + V_y + V_yaw * ( Wheel_base + Wheel_track )/2 )/ Wheel_Radius;
        vel_wheel_expectation[3] = ( V_x + V_y - V_yaw * ( Wheel_base + Wheel_track )/2 )/ Wheel_Radius;
        vel_wheel_expectation[4] = ( V_x - V_y + V_yaw * ( Wheel_base + Wheel_track )/2 )/ Wheel_Radius;
    }


    void ChassisController::calculation_chassis_vel() {
        Vx_chassis = ( vel_wheel_real[1] + vel_wheel_real[2] + vel_wheel_real[3] + vel_wheel_real[4] ) * Wheel_Radius / 4;
        Vy_chassis = (-vel_wheel_real[1] + vel_wheel_real[2] + vel_wheel_real[3] - vel_wheel_real[4] ) * Wheel_Radius / 4;
        Vyaw_chassis = (-vel_wheel_real[1] + vel_wheel_real[2] - vel_wheel_real[3] + vel_wheel_real[4]) * Wheel_Radius / 2 / (Wheel_track + Wheel_base);
    }  

    void ChassisController::Transform_broadcast() {
        //Convert to seconds
        dt = (now_time - last_time).toSec();

        //compute odometry in a typical way given the velocities of the robot
        double delta_displacement_x = (Vx_chassis * cos(theta) - Vy_chassis * sin(theta)) * dt;
        double delta_displacement_y = (Vx_chassis * sin(theta) + Vy_chassis * cos(theta)) * dt;
        double delta_theta = Vyaw_chassis * dt;
        displacement_x += delta_displacement_x;
        displacement_y += delta_displacement_y;
        theta += delta_theta;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //publish the transform over tf
        odom_trans.header.stamp = now_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = displacement_x;
        odom_trans.transform.translation.y = displacement_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
    }

    void ChassisController::Odometry_publish() {
        odom.header.stamp = now_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = displacement_x;
        odom.pose.pose.position.y = displacement_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = Vx_chassis;
        odom.twist.twist.linear.y = Vy_chassis;
        odom.twist.twist.angular.z = Vyaw_chassis;

        //publish the message
        pub.publish(odom);
    }
    void ChassisController::TF_SpeedControl(){
        if (ChassisSpeedMode) {
            stamped_in.header.frame_id = "odom";
            stamped_in.header.stamp = now_time;
            stamped_in.vector.x = V_x;
            stamped_in.vector.y = V_y;
            stamped_in.vector.z = 0.0;

            //Wait for connection between two transformers; ros::Duration(3.0)：Blocking wait timeout
            //ros::Time(0)：Use the latest tf data in the buffer
            listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("base_link", "odom", ros::Time(0), transform);
            listener.transformVector("base_link", stamped_in, stamped_out);
            V_x = stamped_out.vector.x;
            V_y = stamped_out.vector.y;
        }
    }

}// namespace chassis_controller

PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)