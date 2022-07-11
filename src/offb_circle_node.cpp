/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

//////////////////////// GLOBAL VARS ///////////////////////////
enum CONTROL_MODE {POSITION, VELOCITY, ATTITUDE_TARGET};
CONTROL_MODE control_mode = POSITION;
mavros_msgs::State current_state;

ros::Publisher pub_vel;
ros::Publisher attitude_raw_pub_;
geometry_msgs::TwistStamped vel_target_stamped;
int current_extended_state;

geometry_msgs::PoseStamped init_pose;
geometry_msgs::PoseStamped path_pose;
bool init_pose_test = false;
float CYCLES = 1.0;
float RUN_TIME = 12.5; //seconds
float RATE = 20.0;
float tol = .5;
float radius = 3;
float desired_hgt = 5;
float actual_hgt; 
float a = 10;	
float b = 20;
float A = 5;	//For Lissajous
float B = 10;
float delta = M_PI/2;
enum PATH_MODE {CIRCLE, LEMNISCATE, LISSAJOUS};
PATH_MODE path_mode = LEMNISCATE;

//////////////////////// FUNCTIONS /////////////////////////////
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

void set_rotation(double yaw){	//function to convert from angular representation to quaternion
	tf::Quaternion q_des_transformed;
    q_des_transformed.setRPY(0.0, 0.0, yaw);
    path_pose.pose.orientation.w = q_des_transformed.w();
    path_pose.pose.orientation.x = q_des_transformed.x();
    path_pose.pose.orientation.y = q_des_transformed.y();
    path_pose.pose.orientation.z = q_des_transformed.z();
}

void circle(float count_step){
	//int num_points = RUN_TIME * RATE; //sec * Hz <- pt/sec
	//float ang_step = 2*M_PI / num_points;
	float ang;
	float time = count_step * 1/RATE;
	ROS_WARN("TIME: %f", time);
	ROS_WARN("COUNT: %f", count_step);
	float offset; 
	float freq = .5;
	
	switch(path_mode){
		case CIRCLE:
			offset = -.3;
			path_pose.pose.position.x = radius * cos(freq*time);
			path_pose.pose.position.y = radius * sin(freq*time);
			path_pose.pose.position.z = desired_hgt;
			ang = atan2(radius*freq*cos(freq*time), -radius*freq*sin(freq*time)) + offset;
			//ang = ang_step * count;
			//ang = 0;
			break;

		case LEMNISCATE:
			offset = -.75;
			path_pose.pose.position.x = a*cos(time)/(1 + pow(sin(time), 2));
			path_pose.pose.position.y = b*sin(time)*cos(time)/(1 + pow(sin(time), 2));
			path_pose.pose.position.z = desired_hgt;
			ang = atan2(  -a*( pow(sin(time),4)+(pow(cos(time),2)+1)*pow(sin(time),2)-pow(cos(time), 2) )/pow(pow(sin(time), 2)+1, 2)  ,  -a*( pow(sin(time), 4)+(pow(cos(time), 2)+1)*pow(sin(time), 2)-pow(cos(time), 2) ) / pow(pow(sin(time), 2)+1, 2)  ) + offset;
			//ang = 0;
			break;

		case LISSAJOUS:
			a = 1;
			b = 2;
			offset = -.8;
			path_pose.pose.position.x = A*sin(a*time+delta);
			path_pose.pose.position.y = B*sin(b*time);
//			ang = atan2(path_pose.pose.position.y, path_pose.pose.position.x);
			ang = atan2(B*b*cos(b*time), A*a*cos(a*time+delta)) + offset;
			//ang = 0;
			path_pose.pose.position.z = desired_hgt;
			break;
	}
	//ROS_WARN("ROTATING: %f", ang);
	//ROS_WARN("X: %f", radius * cos(1/2*time));
	//ROS_WARN("Y: %f", radius * sin(1/2*time));
	//path_pose.pose.orientation.z = init_pose.pose.orientation.z + ang + M_PI/2;
	//path_pose.pose.orientation.z = ang;

	//pub_att_target(0, 0, 10, ang + M_PI/2, 0);
	set_rotation(ang);
}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
	current_extended_state = msg->landed_state;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg1){
	if(!init_pose_test){
		//init_pose.pose.position.x = msg1->pose.pose.position.x;
		//init_pose.pose.position.y = msg1->pose.pose.position.y;
		init_pose.pose.position = msg1->pose.pose.position;
		init_pose.pose.orientation = msg1->pose.pose.orientation;
		init_pose_test = true;
	}

	actual_hgt = msg1->pose.pose.position.z;
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    // Subcribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/odometry/in", 10, odom_cb);
	//ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::State>
    //        ("mavros/state", 10, extended_state_cb);
    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    pub_vel = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    attitude_raw_pub_ =
            nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	//double RATE = 20.0;
    ros::Rate rate(RATE);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Goal pose for position control
    geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "map";		//add frame to visualize end goal point in RVIZ
    pose.pose.position.x = init_pose.pose.position.x;
    pose.pose.position.y = init_pose.pose.position.y;
    pose.pose.position.z = desired_hgt;
    pose.pose.orientation.x = init_pose.pose.orientation.x;
    pose.pose.orientation.y = init_pose.pose.orientation.y;
    pose.pose.orientation.z = init_pose.pose.orientation.z;
    pose.pose.orientation.w = init_pose.pose.orientation.w;


    // Send a few setpoints before starting. Otherwise, the quadrotor will be disarmed.
    for(int i = 100; ros::ok() && i > 0; --i){
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

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

	bool first_test = false;
	bool mission_complete = false;
	float count = 0.0;

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){

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
                }
                last_request = ros::Time::now();
            }
        }
        // If the quadrotor is successfully armed, send control command.
		if(count < CYCLES*RUN_TIME*RATE){        
			switch (control_mode){
				case POSITION:
					for(int i = 100; ros::ok() && i > 0; --i){
						circle(count);
						local_pos_pub.publish(path_pose);
					}
					
					if (current_state.mode == "OFFBOARD" && current_state.armed)
						if(actual_hgt > desired_hgt-tol && actual_hgt < desired_hgt+tol)
							count = count + 1.0;
					
					break;
				default:
					local_pos_pub.publish(pose);
					count = count + 1.0;
		    }
		}

		if(count == CYCLES*RUN_TIME*RATE){
			ROS_WARN("COMMANDS ALL DONE!!!!!");
			mission_complete = true;
		}	
		
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
