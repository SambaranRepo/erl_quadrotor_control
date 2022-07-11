/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

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


//////////////////////// GLOBAL VARS ///////////////////////////
enum CONTROL_MODE {POSITION, VELOCITY, ATTITUDE_TARGET, GEOMETRIC};
CONTROL_MODE control_mode = GEOMETRIC;
mavros_msgs::State current_state;

ros::Publisher pub_vel;
ros::Publisher attitude_raw_pub_;
ros::Publisher pub_pos_cmd;
ros::Publisher  pub_enable_motors;
geometry_msgs::TwistStamped vel_target_stamped;
kr_mav_msgs::PositionCommand pos_cmd; // Position + Yaw for SO3 Controller.
std_msgs::Bool enable_motors;
//////////////////////// FUNCTIONS /////////////////////////////


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
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

void pub_pos_cmd_geoctrl(double des_x, double des_y, double des_z, double des_yaw)
{
  pos_cmd.position.x = des_x;
  pos_cmd.position.y = des_y;
  pos_cmd.position.z = des_z;
  pos_cmd.velocity.x = 0;
  pos_cmd.velocity.y = 0;
  pos_cmd.velocity.z = 0;
  pos_cmd.acceleration.x = 0;
  pos_cmd.acceleration.y = 0;
  pos_cmd.acceleration.z = 0;
  pos_cmd.jerk.x = 0;
  pos_cmd.jerk.y = 0;
  pos_cmd.jerk.z = 0;

  pos_cmd.yaw = des_yaw;
  pos_cmd.yaw_dot = 0;
  pub_pos_cmd.publish(pos_cmd);
  ROS_INFO("pub_pos_cmd_geoctrl called");
  // By default, the gains from the message is ignored by the controller.
  //pos_cmd.kx[0] = kx[0], pos_cmd.kx[1] = kx[1], pos_cmd.kx[2] = kx[2];
  //pos_cmd.kv[0] = kv[0], pos_cmd.kv[1] = kv[1], pos_cmd.kv[2] = kv[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_geoctl_node");
    ros::NodeHandle nh;
    // Subcribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
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

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Goal pose for position control
    geometry_msgs::PoseStamped pose;
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

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

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
                } else {
					enable_motors.data = false;
					pub_enable_motors.publish(enable_motors);
                last_request = ros::Time::now();
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
                	pub_pos_cmd_geoctrl(2.0, 2.0, 5.0, 1.57);
				} else {
					local_pos_pub.publish(pose);
				}
                break;
            default:
                local_pos_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
