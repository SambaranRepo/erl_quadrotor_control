/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <kr_mav_msgs/PositionCommand.h>
#include "erl_quadrotor_control/panelcmd.h"


//////////////////////// GLOBAL VARS ///////////////////////////
enum CONTROL_MODE {POSITION, VELOCITY, ATTITUDE_TARGET, GEOMETRIC};
CONTROL_MODE control_mode = GEOMETRIC;
mavros_msgs::State current_state;

ros::Publisher pub_vel;
ros::Publisher attitude_raw_pub_;
ros::Publisher pub_pos_cmd;
ros::Publisher  pub_enable_motors;
geometry_msgs::TwistStamped vel_target_stamped;
kr_mav_msgs::PositionCommand ros_pos_cmd; // Position + Yaw for SO3 Controller.
std_msgs::Bool enable_motors;

erl_quadrotor_control::panelcmd panel_cmd; 
geometry_msgs::PoseStamped pose;
bool takeoff_occured = false; 
float RUN_TIME = 12.5; //seconds
float RATE = 20.0;
//////////////////////// FUNCTIONS /////////////////////////////


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void panel_cb(const erl_quadrotor_control::panelcmd::ConstPtr& msg){
    panel_cmd = *msg; 

    // Check if Takeoff command is received 
    if (panel_cmd.arm == true){
        takeoff_occured = true; 
    }

    // Update control_mode
    if (panel_cmd.algorithm == 2){
        control_mode = GEOMETRIC; 
    }
    else {
        control_mode = POSITION;
    }

    // Update the desired setpoints 
    pose.pose.position.x = panel_cmd.x;
    pose.pose.position.y = panel_cmd.y;
    pose.pose.position.z = panel_cmd.z;
}

void pub_lin_vel(double x, double y, double z) {
  geometry_msgs::Twist vel_target;
  geometry_msgs::Vector3 vel_lin_target;
  geometry_msgs::Vector3 vel_ang_target;

  vel_target_stamped.header.stamp = ros::Time::now();
  vel_target_stamped.header.seq++;

  vel_lin_target.x = x;
  vel_lin_target.y = y;
  vel_lin_target.z = z;

  vel_ang_target.x = 0;
  vel_ang_target.y = 0;
  vel_ang_target.z = 0; //yaw

  vel_target.linear = vel_lin_target;
  vel_target.angular = vel_ang_target;
  vel_target_stamped.twist = vel_target;

  pub_vel.publish(vel_target_stamped);
}

std::vector<double> tracker(double t)
{
    std::vector<double> pos_cmd;
    // For a Hover 
    if (panel_cmd.trajectory_mode == 1)
    {
        pos_cmd.push_back(0.0); // x
        pos_cmd.push_back(0.0); // y
        pos_cmd.push_back(5.0); // z
        pos_cmd.push_back(0.0); // vx
        pos_cmd.push_back(0.0); // vy
        pos_cmd.push_back(0.0); // vz
        pos_cmd.push_back(0.0); // ax
        pos_cmd.push_back(0.0); // ay
        pos_cmd.push_back(0.0); // az
        pos_cmd.push_back(0.0); // jx
        pos_cmd.push_back(0.0); // jy
        pos_cmd.push_back(0.0); // jz
        pos_cmd.push_back(0.0); // yaw
        pos_cmd.push_back(0.0); // yawdot
    }
    // For a Circle 
    else if (panel_cmd.trajectory_mode == 2)
    {
        double T = 5; // time to finish a loop
        double radius = 2;
        double cx = -2, cy = 0;
        // tracking a circle
        pos_cmd.push_back(cx + radius*cos(2*M_PI*t/T)); // x
        pos_cmd.push_back(cy + radius*sin(2*M_PI*t/T)); // y
        pos_cmd.push_back(5.0); // z
        pos_cmd.push_back(-radius*(2*M_PI/T)*sin(2*M_PI*t/T)); // vx
        pos_cmd.push_back(radius*(2*M_PI/T)*cos(2*M_PI*t/T)); // vy
        pos_cmd.push_back(0.0); // vz
        pos_cmd.push_back(0.0); // ax
        pos_cmd.push_back(0.0); // ay
        pos_cmd.push_back(0.0); // az
        pos_cmd.push_back(0.0); // jx
        pos_cmd.push_back(0.0); // jy
        pos_cmd.push_back(0.0); // jz
        pos_cmd.push_back(1.57); // yaw
        pos_cmd.push_back(0.0); // yawdot
    }
    // For a setpoint 
    else if (panel_cmd.trajectory_mode == 3)
    {
        pos_cmd.push_back(panel_cmd.x); // x
        pos_cmd.push_back(panel_cmd.y); // y
        pos_cmd.push_back(panel_cmd.z); // z
        pos_cmd.push_back(0.0); // vx
        pos_cmd.push_back(0.0); // vy
        pos_cmd.push_back(0.0); // vz
        pos_cmd.push_back(0.0); // ax
        pos_cmd.push_back(0.0); // ay
        pos_cmd.push_back(0.0); // az
        pos_cmd.push_back(0.0); // jx
        pos_cmd.push_back(0.0); // jy
        pos_cmd.push_back(0.0); // jz
        pos_cmd.push_back(panel_cmd.yaw); // yaw
        pos_cmd.push_back(0.0); // yawdot
    }
    
    return pos_cmd;
}

void pub_att_target(double vx, double vy, double vz, double yaw, double throttle)
{
    // clamp from 0.0 to 1.0
    throttle = std::min(1.0, throttle);
    throttle = std::max(0.0, throttle);
    auto setpoint_msg = boost::make_shared<mavros_msgs::AttitudeTarget>();
    setpoint_msg->header.stamp = ros::Time::now();
    setpoint_msg->header.seq++;
    setpoint_msg->type_mask = 0;

    tf::Quaternion q_des_transformed;
    q_des_transformed.setRPY(0.0, 0.0, yaw);
    setpoint_msg->orientation.w = q_des_transformed.w();
    setpoint_msg->orientation.x = q_des_transformed.x();
    setpoint_msg->orientation.y = q_des_transformed.y();
    setpoint_msg->orientation.z = q_des_transformed.z();
    setpoint_msg->body_rate.x = vx;
    setpoint_msg->body_rate.y = vy;
    setpoint_msg->body_rate.z = vz;
    setpoint_msg->thrust = throttle;

    attitude_raw_pub_.publish(setpoint_msg);
}

void pub_pos_cmd_geoctrl(std::vector<double> pos_cmd)
{
  ros_pos_cmd.position.x = pos_cmd.at(0);
  ros_pos_cmd.position.y = pos_cmd.at(1);
  ros_pos_cmd.position.z = pos_cmd.at(2);
  ros_pos_cmd.velocity.x = pos_cmd.at(3);
  ros_pos_cmd.velocity.y = pos_cmd.at(4);
  ros_pos_cmd.velocity.z = pos_cmd.at(5);
  ros_pos_cmd.acceleration.x = pos_cmd.at(6);
  ros_pos_cmd.acceleration.y = pos_cmd.at(7);
  ros_pos_cmd.acceleration.z = pos_cmd.at(8);
  ros_pos_cmd.jerk.x = pos_cmd.at(9);
  ros_pos_cmd.jerk.y = pos_cmd.at(10);
  ros_pos_cmd.jerk.z = pos_cmd.at(11);

  ros_pos_cmd.yaw = pos_cmd.at(12);
  ros_pos_cmd.yaw_dot = pos_cmd.at(13);
  pub_pos_cmd.publish(ros_pos_cmd);
  ROS_INFO("pub_pos_cmd_geoctrl called");
  // By default, the gains from the message is ignored by the controller.
  //pos_cmd.kx[0] = kx[0], pos_cmd.kx[1] = kx[1], pos_cmd.kx[2] = kx[2];
  //pos_cmd.kv[0] = kv[0], pos_cmd.kv[1] = kv[1], pos_cmd.kv[2] = kv[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_panel_node");
    ros::NodeHandle nh;
    // Subcribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber panel_sub = nh.subscribe<erl_quadrotor_control::panelcmd>
            ("erl_quadrotor_control/quadrotor_gui_cmd", 10, panel_cb);
    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    pub_vel = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    attitude_raw_pub_ =
            nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
	pub_pos_cmd = nh.advertise<kr_mav_msgs::PositionCommand>("position_cmd", 2, true);

	pub_enable_motors = nh.advertise<std_msgs::Bool>("/motors", 2, true);
    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate(20.0);

    // Wait for Control Panel Settings
    while(panel_cmd.arm == false || 
    panel_cmd.algorithm == 0 || panel_cmd.trajectory_mode == 0){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for all settings from Control Panel");
    }

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Goal pose for position control
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0.707;
    pose.pose.orientation.w = 0.707;

    // Send a few setpoints before starting. Otherwise, the quadrotor will be disarmed.
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set offboard mode, i.e. autonomous mode.
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //Set mode to AUTO.LAND when mission complete
	mavros_msgs::SetMode autol_set_mode;
	autol_set_mode.request.base_mode = 0;
	autol_set_mode.request.custom_mode = "AUTO.LAND";
    bool mission_complete = false;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	

	bool start_tracking = false;
	ros::Time start_time;

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        // Try to enable offboard mode
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            // Try to arm the quadrotor
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
					enable_motors.data = true;
					pub_enable_motors.publish(enable_motors);
//					start_tracking = false;				
                } else {
					enable_motors.data = false;
					pub_enable_motors.publish(enable_motors);
                	last_request = ros::Time::now();
					if (!start_tracking)
					{
						start_tracking = true;
						start_time = ros::Time::now();
					}
				}
            }
        }
        // If the quadrotor is successfully armed, send control command.
        switch (control_mode)
        {
            case POSITION:
                local_pos_pub.publish(pose);
                break;
            case VELOCITY:
                pub_lin_vel(0.0, 0.0, 0.5);
                break;
            case ATTITUDE_TARGET:
                pub_att_target(0.0, 0.0, 0.0, 1.57, 0.6);
                break;
            case GEOMETRIC:
				if (current_state.armed) {
					double t = (ros::Time::now() - start_time).toSec();
					auto pos_cmd = tracker(t);
                	pub_pos_cmd_geoctrl(pos_cmd);
				} else {
					local_pos_pub.publish(pose);
				}
                break;
            default:
                local_pos_pub.publish(pose);
        }
        
        ros::spinOnce();
        rate.sleep();

        // Check if land command from control panel is received 
        if (takeoff_occured == true && panel_cmd.arm == false){
            mission_complete = true; 
        }

        if(mission_complete){
			if( set_mode_client.call(autol_set_mode) && autol_set_mode.response.mode_sent){
		            ROS_WARN("AUTO.LAND enabled");
					
					for(int i = 0; ros::ok() && i < RUN_TIME * RATE; ++i){
						ros::spinOnce();
						rate.sleep();
					}
					
									
					ROS_WARN(current_state.mode.c_str());
					ROS_WARN("SHUTTING DOWN.");
					ros::shutdown();
			}
		}
    }

    return 0;
}
