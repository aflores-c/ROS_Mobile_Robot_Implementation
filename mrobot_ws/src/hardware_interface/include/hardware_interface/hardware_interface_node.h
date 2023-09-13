#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <angles/angles.h>

//messages types for publishing and subscribing
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
static const double ENC_FACTOR = 0.000872667;


class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
public:
ROBOTHardwareInterface(ros::NodeHandle& nh);
~ROBOTHardwareInterface();
void init();
void update(const ros::TimerEvent& e);
void read(ros::Duration elapsed_time); //to receive data from Arduino
void write(ros::Duration elapsed_time); //to send command vel to Arduino
ros::Publisher wheel_vel_pub; //publish to /set_vel
ros::Subscriber wheel_enc_sub; //suscriber to /encoder_ticks
void enc_ticks_CB(const std_msgs::Int64MultiArray::ConstPtr &enc_msg); //subscriber callback


protected:
//interface to receive data or status of the robot (encoders)
hardware_interface::JointStateInterface joint_state_interface_;
//interface to send velocity commands to the robot
hardware_interface::VelocityJointInterface velocity_joint_interface_;
//interface to set the velocity limits of the robot
joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;

std::string joint_name_[4]={"lf_wheel_joint","rf_wheel_joint","lb_wheel_joint","rb_wheel_joint"};
double joint_position_degrees_[4]; //Position from Arduino in degrees
double joint_position_[4];//Position in radians
double joint_velocity_command_[2]; //To send velocity commands to Arduino
std_msgs::Float32MultiArray vel_setpoint_cmd_array;
double joint_velocity_[4]; //Not used
double joint_effort_[4]; //Not used
int curr_enc[4] = {0, 0, 0, 0};
int prev_enc[4] = {0, 0, 0, 0};

ros::NodeHandle nh_;
ros::Timer non_realtime_loop_;
ros::Duration elapsed_time_;
double loop_hz_;
boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

};