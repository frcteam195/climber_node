#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <map>
#include "hmi_agent_node/HMI_Signals.h"
#include "ck_utilities/Motor.hpp"
#include "ck_utilities/Piston.hpp"
#include "rio_control_node/Robot_Status.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Motor_Info.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


#define LEFT_CLIMBER_MASTER_CAN_ID 12
#define LEFT_CLIMBER_FOLLOWER_CAN_ID 13
#define RIGHT_CLIMBER_MASTER_CAN_ID 14
#define RIGHT_CLIMBER_FOLLOWER_CAN_ID 15

#define CLIMBER_SOLENOID_ID 4

ros::NodeHandle* node;

static constexpr double CLIMBER_BALANCE_GAIN = 0.0795;

static Motor* left_climber_master;
static Motor* left_climber_follower;
static Motor* right_climber_master;
static Motor* right_climber_follower;

static Piston* climber_solenoid;

static double imu_roll_rad;

enum class ClimberStates
{

	IDLE,
	DEPLOY_INITIAL_HOOKS,
	RETRACT_HOOKS,
	GRAB_INITIAL_BAR,
	PULL_UP,
	STATIC_LATCH,
	GRAB_NEXT_BAR_INITIAL_UNWINCH,
	GRAB_NEXT_BAR_EXTEND_PISTONS,
	GRAB_NEXT_BAR_UNWINCH_COMPLETELY,
	GRAB_NEXT_BAR_RETRACT_PISTONS,
	GRAB_NEXT_BAR_PULL_UP,
	STATIC_UNLATCH,
	END,
	STOPPED

};

static ClimberStates climber_state = ClimberStates::IDLE;
static ClimberStates next_climber_state = ClimberStates::IDLE;

static bool stop_climber = false;
static bool deploy_hooks = false;
static bool begin_climb = false;
static bool retract_hooks = false;

static std::map<uint16_t, rio_control_node::Motor_Info> motor_status_map;
static double left_climber_position = 0.0;
static double right_climber_position = 0.0;

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    stop_climber = msg.stop_climber || stop_climber;
	deploy_hooks = msg.deploy_hooks;
	begin_climb = msg.begin_climb;
	retract_hooks = msg.retract_hooks;
}

void motor_status_callback(const rio_control_node::Motor_Status& msg)
{
	for ( const rio_control_node::Motor_Info& motorInfo : msg.motors )
	{
		motor_status_map[motorInfo.id] = motorInfo;
	}

	if ( motor_status_map.count(LEFT_CLIMBER_MASTER_CAN_ID) && motor_status_map.count(RIGHT_CLIMBER_MASTER_CAN_ID) )
	{
		left_climber_position = motor_status_map [LEFT_CLIMBER_MASTER_CAN_ID].sensor_position;
		right_climber_position = motor_status_map [RIGHT_CLIMBER_MASTER_CAN_ID].sensor_position;
	}
	else
	{
		left_climber_position = 0.0;
		right_climber_position = 0.0;
	}
}

void imu_sensor_callback(const nav_msgs::Odometry& msg)
{
	tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
	msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	tf2Scalar roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	imu_roll_rad = roll;
	(void) pitch;
	(void) yaw;
}

void auto_balance_climb(double target_position)
{
	double arbFF = CLIMBER_BALANCE_GAIN * imu_roll_rad;
	left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, target_position, arbFF);
	right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, target_position, -arbFF);
}

void step_state_machine()
{
	static ros::Time time_state_entered = ros::Time::now();

	if(climber_state != next_climber_state)
	{
		time_state_entered = ros::Time::now();
	}

	ros::Duration time_in_state = ros::Time::now() - time_state_entered;

	climber_state = next_climber_state;

	switch(climber_state)
	{
		case ClimberStates::IDLE: //Stop all motors
		{
			left_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			right_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			climber_solenoid->set(Piston::PistonState::OFF);
		
			if(deploy_hooks)
			{
				climber_state = ClimberStates::DEPLOY_INITIAL_HOOKS;
			}

			break;
		}

		case ClimberStates::DEPLOY_INITIAL_HOOKS: //Winch up hooks
		{
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 15, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 15, 0);

			if(left_climber_position > 14.5 && right_climber_position > 14.5)
			{
				climber_state = ClimberStates::GRAB_INITIAL_BAR;
			}
			break;
		}

		case ClimberStates::GRAB_INITIAL_BAR: //no op
		{
			climber_state = ClimberStates::PULL_UP;
			break;
		}

		case ClimberStates::PULL_UP: //Winch down hooks
		{
			auto_balance_climb(4.0);

			if(left_climber_position < 4.5 && right_climber_position < 4.5)
			{
				climber_state = ClimberStates::STATIC_LATCH;
			}
			break;
		}

		case ClimberStates::STATIC_LATCH://Continue to pull up
		{
			climber_state = ClimberStates::GRAB_NEXT_BAR_INITIAL_UNWINCH;
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_INITIAL_UNWINCH://unwinch a little bit
		{
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 6.0, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 6.0, 0);

			if(right_climber_position > 5.5 && left_climber_position > 5.5)
			{
				climber_state = ClimberStates::GRAB_NEXT_BAR_EXTEND_PISTONS;
			}
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_EXTEND_PISTONS://activate pistons, 
		{
			climber_solenoid->set(Piston::PistonState::ON);

			if()
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_UNWINCH_COMPLETELY://uwinch a lot
		{
			
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 15, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 15, 0);
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_RETRACT_PISTONS://actuate pistons
		{
			climber_solenoid->set(Piston::PistonState::OFF);
			break;
		}


		case ClimberStates::GRAB_NEXT_BAR_PULL_UP://pull up
		{
			auto_balance_climb(4.0);
			break;
		}

		case ClimberStates::STATIC_UNLATCH://no op
		{
			break;
		}

		case ClimberStates::END://End
		{
			//deploy fireworks
			break;
		}

		case ClimberStates::RETRACT_HOOKS://Winch down hooks, go to idle
		{
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 4.0, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 4.0, 0);
			break;
		}

		case ClimberStates::STOPPED://Turn off all motors and stay there
		{
			
			break;
		}
	}

	
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "climber_node");

	ros::NodeHandle n;

	node = &n;

	ros::Subscriber hmi_subscribe = node->subscribe("/HMISignals", 1, hmi_signal_callback);
	ros::Subscriber imu_subscribe = node->subscribe("/RobotIMU", 1, imu_sensor_callback);
	ros::Subscriber motor_status_subscribe = node->subscribe("/MotorStatus", 1, motor_status_callback);

	left_climber_master = new Motor(LEFT_CLIMBER_MASTER_CAN_ID, Motor::Motor_Type::TALON_FX);
	right_climber_master = new Motor(RIGHT_CLIMBER_MASTER_CAN_ID, Motor::Motor_Type::TALON_FX);
	left_climber_follower = new Motor(LEFT_CLIMBER_FOLLOWER_CAN_ID, Motor::Motor_Type::TALON_FX);
	right_climber_follower = new Motor(RIGHT_CLIMBER_FOLLOWER_CAN_ID, Motor::Motor_Type::TALON_FX);

	climber_solenoid = new Piston(CLIMBER_SOLENOID_ID, Piston::PistonType::SINGLE);


	ros::Rate rate(100);

	while(ros::ok())
	{
		ros::spinOnce();
		step_state_machine();
		rate.sleep();
	}

	return 0;
}