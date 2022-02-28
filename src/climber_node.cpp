#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
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
	GRAB_NEXT_BAR,
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

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    stop_climber = msg.stop_climber || stop_climber;
	deploy_hooks = msg.deploy_hooks;
	begin_climb = msg.begin_climb;
	retract_hooks = msg.retract_hooks;
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

void auto_balance_climb()
{

}

void step_state_machine()
{
	climber_state = next_climber_state;

	switch(climber_state)
	{
		case ClimberStates::IDLE: //Stop all motors
		{
			left_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			right_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			climber_solenoid->set(Piston::PistonState::OFF);
			break;
		}

		case ClimberStates::DEPLOY_INITIAL_HOOKS: //Winch up hooks
		{
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, -1.5, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, -1.5, 0);
			break;
		}

		case ClimberStates::GRAB_INITIAL_BAR: //no op
		{
			break;
		}

		case ClimberStates::PULL_UP: //Winch down hooks
		{

			break;
		}

		case ClimberStates::STATIC_LATCH://Continue to pull up
		{
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR:
		//unwinch a little bit, activate pistons, unwinch a lot, actuate pistons, pull up
		{
			break;
		}

		case ClimberStates::STATIC_UNLATCH://no op
		{
			break;
		}

		case ClimberStates::END://End
		{
			break;
		}

		case ClimberStates::RETRACT_HOOKS://Winch down hooks, go to idle
		{
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