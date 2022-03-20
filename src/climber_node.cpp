#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <map>
#include "hmi_agent_node/HMI_Signals.h"
#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/Motor.hpp"
#include "ck_utilities/Solenoid.hpp"
#include "rio_control_node/Robot_Status.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Motor_Info.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "climber_node/Climber_Diagnostics.h"
#include "climber_node/Climber_Status.h"


#define LEFT_CLIMBER_MASTER_CAN_ID 12
// #define LEFT_CLIMBER_FOLLOWER_CAN_ID 13
#define RIGHT_CLIMBER_MASTER_CAN_ID 14
// #define RIGHT_CLIMBER_FOLLOWER_CAN_ID 15

#define CLIMBER_MOVEABLE_ARM_SOLENOID_ID 6
#define CLIMBER_STATIC_HOOKS_SOLENOID_ID 7

ros::NodeHandle* node;

static constexpr double CLIMBER_BALANCE_GAIN = 0.0795;
static constexpr double CLIMBER_FULL_RETRACTION = 0;
static constexpr double CLIMBER_HANDOFF_HEIGHT = (5341.0 / 2048.0) / 20.0;
static constexpr double CLIMBER_WAIT_HEIGHT = (160689.0 / 2048.0) / 20.0;
static constexpr double CLIMBER_INITIAL_GRAB_HEIGHT = (318150.0 / 2048.0) / 20.0;
static constexpr double CLIMBER_PARTIAL_RELEASE_HEIGHT = (97875.0 / 2048.0) / 20.0;
static constexpr double CLIMBER_MAX_EXTENSION = (393379.0 / 2048.0) / 20.0;
static constexpr double CLIMBER_MAX_LIMIT = (446960.0 / 2048.0) / 20.0;
static constexpr double CLIMBER_HEIGHT_DELTA = 0.25;


static Motor* left_climber_master;
// static Motor* left_climber_follower;
static Motor* right_climber_master;
// static Motor* right_climber_follower;

static Solenoid* climber_arm_solenoid;
static Solenoid* climber_static_hooks_solenoid;

static double imu_roll_rad = 0;
static double imu_pitch_rad = 0;
static double imu_pitch_rad_per_sec = 0;
static double prev_imu_pitch_rad = 0;
static ros::Time prev_time_imu = ros::Time(0);
static double bar_counter = 0;

enum class ClimberStates : int
{

	IDLE,
	DEPLOY_INITIAL_HOOKS,
	GRAB_INITIAL_BAR,
	PULL_UP,
	STATIC_LATCH,
	GRAB_NEXT_BAR_INITIAL_UNWINCH,
	GRAB_NEXT_BAR_EXTEND_PISTONS,
	GRAB_NEXT_BAR_UNWINCH_COMPLETELY,
	GRAB_NEXT_BAR_RETRACT_PISTONS,
	GRAB_NEXT_BAR_PULL_UP,
	STATIC_UNLATCH,
	FINISH_WINCHING,
	WAIT_FOR_STABLE,
	END,
	STOPPED

};

std::map<ClimberStates,std::string> climber_state_lookup = 
{
	{ClimberStates::IDLE, "IDLE"},
	{ClimberStates::DEPLOY_INITIAL_HOOKS, "DEPLOY_INITITAL_HOOKS"},
	{ClimberStates::GRAB_INITIAL_BAR, "GRAB_INITIAL_BAR"},
	{ClimberStates::PULL_UP, "PULL_UP"},
	{ClimberStates::STATIC_LATCH, "STATIC_LATCH"},
	{ClimberStates::GRAB_NEXT_BAR_INITIAL_UNWINCH, "GRAB_NEXT_BAR_INITIAL_UNWINCH"},
	{ClimberStates::GRAB_NEXT_BAR_EXTEND_PISTONS, "GRAB_NEXT_BAR_EXTEND_PISTONS"},
	{ClimberStates::GRAB_NEXT_BAR_UNWINCH_COMPLETELY, "GRAB_NEXT_BAR_UNWINCH_COMPLETLY"},
	{ClimberStates::GRAB_NEXT_BAR_RETRACT_PISTONS, "GRAB_NEXT_BAR_RETRACT_PISTONS"},
	{ClimberStates::GRAB_NEXT_BAR_PULL_UP, "GRAB_NEXT_BAR_PULL_UP"},
	{ClimberStates::STATIC_UNLATCH, "STATIC_UNLATCH"},
	{ClimberStates::FINISH_WINCHING, "FINISH_WINCHING"},
	{ClimberStates::WAIT_FOR_STABLE, "WAIT_FOR_STABLE"},
	{ClimberStates::END, "END"},
	{ClimberStates::STOPPED, "STOPPED"}
};

static ClimberStates climber_state = ClimberStates::IDLE;
static ClimberStates next_climber_state = ClimberStates::IDLE;

static bool stop_climber = false;
static bool deploy_hooks = false;
static bool begin_climb = false;
static bool retract_hooks = false;
static bool forced_reset_retract_hooks = false;

static std::map<uint16_t, rio_control_node::Motor_Info> motor_status_map;
static double left_climber_position = 0.0;
static double right_climber_position = 0.0;


std::string climber_state_to_string(ClimberStates state)
{
    if(climber_state_lookup.count(state))
	{
		return climber_state_lookup[state];
	}
	return "INVALID";
}


void publish_diagnostic_data()
{
    static ros::Publisher diagnostic_publisher = node->advertise<climber_node::Climber_Diagnostics>("/ClimberNodeDiagnostics", 1);
    climber_node::Climber_Diagnostics diagnostics;
    diagnostics.climber_state = climber_state_to_string(climber_state);
    diagnostics.next_climber_state = climber_state_to_string(next_climber_state);
    diagnostics.stop_climber = stop_climber;
    diagnostics.deploy_hooks = deploy_hooks;
    diagnostics.begin_climb = begin_climb;
    diagnostics.retract_hooks = retract_hooks;
    diagnostics.left_climber_position = left_climber_position;
    diagnostics.right_climber_position = right_climber_position;
	diagnostics.forced_reset_retract_hooks = forced_reset_retract_hooks;
	diagnostics.pitch = imu_pitch_rad;
	diagnostics.pitch_rad_per_sec = imu_pitch_rad_per_sec;
    diagnostic_publisher.publish(diagnostics);
}

void hmi_signal_callback(const hmi_agent_node::HMI_Signals& msg)
{
    stop_climber = msg.stop_climber || stop_climber;
	deploy_hooks = msg.deploy_hooks;
	begin_climb = msg.begin_climb;
	retract_hooks = msg.retract_hooks;
	forced_reset_retract_hooks = msg.forced_reset_retract_hooks;
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
	static double averager = 0;
	tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
	msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	tf2Scalar roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	imu_roll_rad = roll;
	imu_pitch_rad = pitch;
	if (prev_time_imu != ros::Time(0))
	{
		ros::Duration dt = ros::Time::now() - prev_time_imu;
		averager = ((imu_pitch_rad - prev_imu_pitch_rad) / (dt.toNSec()/1000000000.0));
		imu_pitch_rad_per_sec = (imu_pitch_rad_per_sec + averager)/2.0;
	}
	else
	{
		imu_pitch_rad_per_sec = 0;
	}
	prev_time_imu = ros::Time::now();
	prev_imu_pitch_rad = pitch;
	(void) yaw;
}

void auto_balance_climb(double target_position)
{
	//double arbFF = CLIMBER_BALANCE_GAIN * imu_roll_rad;
	double arbFF = 0;
	left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, target_position, arbFF);
	right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, target_position, -arbFF);
}

void step_state_machine()
{
	static ros::Time time_state_entered = ros::Time::now();
	static ros::Publisher climber_status_publisher = node->advertise<climber_node::Climber_Status>("/ClimberStatus", 1);
	static climber_node::Climber_Status climber_status;

	if(climber_state != next_climber_state)
	{
		time_state_entered = ros::Time::now();
	}
	climber_state = next_climber_state;

	ros::Duration time_in_state = ros::Time::now() - time_state_entered;

	if(stop_climber)
	{
		climber_state = ClimberStates::STOPPED;
	}
	if (forced_reset_retract_hooks)
	{
		climber_state = ClimberStates::IDLE;
	}
	switch(climber_state)
	{
		case ClimberStates::IDLE: //Stop all motors
		{
			climber_status.climber_retract_intake = false;
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, 0, 0);
			climber_arm_solenoid->set(Solenoid::SolenoidState::ON);
		
			if(deploy_hooks)
			{
				next_climber_state = ClimberStates::DEPLOY_INITIAL_HOOKS;
			}

			break;
		}

		case ClimberStates::DEPLOY_INITIAL_HOOKS: //Winch up hooks
		{
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, CLIMBER_INITIAL_GRAB_HEIGHT, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, CLIMBER_INITIAL_GRAB_HEIGHT, 0);
			climber_static_hooks_solenoid->set(Solenoid::SolenoidState::ON);

			if(ck::math::inRange(CLIMBER_INITIAL_GRAB_HEIGHT - left_climber_position, CLIMBER_HEIGHT_DELTA)
				&& ck::math::inRange(CLIMBER_INITIAL_GRAB_HEIGHT - right_climber_position, CLIMBER_HEIGHT_DELTA))
			{
				climber_arm_solenoid->set(Solenoid::SolenoidState::OFF);
				next_climber_state = ClimberStates::GRAB_INITIAL_BAR;
			}
			break;
		}

		case ClimberStates::GRAB_INITIAL_BAR: //no op
		{
			if(retract_hooks)
			{
				next_climber_state = ClimberStates::IDLE;
			}
			if (begin_climb)
			{
				next_climber_state = ClimberStates::PULL_UP;
			}
			break;
		}

		case ClimberStates::PULL_UP: //Winch down hooks
		{
			auto_balance_climb(CLIMBER_HANDOFF_HEIGHT);

			if(ck::math::inRange(CLIMBER_HANDOFF_HEIGHT - left_climber_position, CLIMBER_HEIGHT_DELTA)
				&& ck::math::inRange(CLIMBER_HANDOFF_HEIGHT - right_climber_position, CLIMBER_HEIGHT_DELTA))
			{
				next_climber_state = ClimberStates::STATIC_LATCH;
			}
			break;
		}

		case ClimberStates::STATIC_LATCH://Continue to pull up
		{
			next_climber_state = ClimberStates::GRAB_NEXT_BAR_INITIAL_UNWINCH;
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_INITIAL_UNWINCH://unwinch a little bit
		{
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, CLIMBER_PARTIAL_RELEASE_HEIGHT, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, CLIMBER_PARTIAL_RELEASE_HEIGHT, 0);
			climber_arm_solenoid->set(Solenoid::SolenoidState::ON);

			if(time_in_state > ros::Duration(0.5) && ck::math::inRange(CLIMBER_PARTIAL_RELEASE_HEIGHT - left_climber_position, CLIMBER_HEIGHT_DELTA)
				&& ck::math::inRange(CLIMBER_PARTIAL_RELEASE_HEIGHT - right_climber_position, CLIMBER_HEIGHT_DELTA))
			{
				next_climber_state = ClimberStates::GRAB_NEXT_BAR_EXTEND_PISTONS;
			}
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_EXTEND_PISTONS://activate solenoids, 
		{
			next_climber_state = ClimberStates::GRAB_NEXT_BAR_UNWINCH_COMPLETELY;
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_UNWINCH_COMPLETELY://uwinch a lot
		{
			
			left_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, CLIMBER_MAX_EXTENSION, 0);
			right_climber_master->set(Motor::Control_Mode::MOTION_MAGIC, CLIMBER_MAX_EXTENSION, 0);
			if(ck::math::inRange(CLIMBER_MAX_EXTENSION - left_climber_position, CLIMBER_HEIGHT_DELTA)
				&& ck::math::inRange(CLIMBER_MAX_EXTENSION - right_climber_position, CLIMBER_HEIGHT_DELTA))
			{
				next_climber_state = ClimberStates::GRAB_NEXT_BAR_RETRACT_PISTONS;
			}
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR_RETRACT_PISTONS://actuate solenoids
		{
			if (bar_counter < 1)
			{
				left_climber_master->config().set_motion_cruise_velocity(7000);
				left_climber_master->config().set_motion_acceleration(12000);
				right_climber_master->config().set_motion_cruise_velocity(7000);
				right_climber_master->config().set_motion_acceleration(12000);
				left_climber_master->config().apply();
				right_climber_master->config().apply();
			}
			climber_arm_solenoid->set(Solenoid::SolenoidState::OFF);
			if (bar_counter > 0)
			{
				climber_status.climber_retract_intake = true;
			}
			if(time_in_state > ros::Duration(0.5) || (time_in_state > ros::Duration(0.5) && bar_counter > 0 && imu_pitch_rad > 0 && imu_pitch_rad_per_sec > 1))
			{
				next_climber_state = ClimberStates::GRAB_NEXT_BAR_PULL_UP;
			}
			break;
		}


		case ClimberStates::GRAB_NEXT_BAR_PULL_UP://pull up
		{
			auto_balance_climb(CLIMBER_WAIT_HEIGHT);
			if(ck::math::inRange(CLIMBER_WAIT_HEIGHT - left_climber_position, CLIMBER_HEIGHT_DELTA)
				&& ck::math::inRange(CLIMBER_WAIT_HEIGHT - right_climber_position, CLIMBER_HEIGHT_DELTA))
			{
				next_climber_state = ClimberStates::STATIC_UNLATCH;
			}
			break;
		}

		case ClimberStates::STATIC_UNLATCH://no op
		{
			if (bar_counter > 0)
			{
				next_climber_state = ClimberStates::END;
			}
			else if (time_in_state > ros::Duration(1) || (time_in_state > ros::Duration(1) && imu_pitch_rad > 0.12 && imu_pitch_rad_per_sec > 1)){
				bar_counter++;
				next_climber_state = ClimberStates::FINISH_WINCHING;
				left_climber_master->config().set_motion_cruise_velocity(14500);
				left_climber_master->config().set_motion_acceleration(22000);
				right_climber_master->config().set_motion_cruise_velocity(14500);
				right_climber_master->config().set_motion_acceleration(22000);
				left_climber_master->config().apply();
				right_climber_master->config().apply();
			}
			break;
		}

		case ClimberStates::FINISH_WINCHING://no op
		{
			auto_balance_climb(CLIMBER_HANDOFF_HEIGHT);
			if(ck::math::inRange(CLIMBER_HANDOFF_HEIGHT - left_climber_position, CLIMBER_HEIGHT_DELTA)
				&& ck::math::inRange(CLIMBER_HANDOFF_HEIGHT - right_climber_position, CLIMBER_HEIGHT_DELTA))
			{
				next_climber_state = ClimberStates::WAIT_FOR_STABLE;
			}
			break;
		}

		case ClimberStates::WAIT_FOR_STABLE://no op
		{
			if (time_in_state > ros::Duration(1.5)){
				next_climber_state = ClimberStates::GRAB_NEXT_BAR_INITIAL_UNWINCH;
			}
			break;
		}

		case ClimberStates::END://End
		{
			//deploy fireworks
			left_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			right_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			break;
		}

		case ClimberStates::STOPPED://Turn off all motors and stay there
		{
			left_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			right_climber_master->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			break;
		}
	}

	climber_status.climber_state = (int8_t) climber_state;
	climber_status_publisher.publish(climber_status);
}

void config_climber_motor(Motor* m)
{
	m->config().set_supply_current_limit(true, 40, 25, 2);
	m->config().set_voltage_compensation_saturation(11);
	m->config().set_voltage_compensation_enabled(true);
	m->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
	m->config().set_forward_soft_limit(CLIMBER_MAX_LIMIT);
	m->config().set_forward_soft_limit_enable(true);
	m->config().set_reverse_soft_limit(-0.2);
	m->config().set_reverse_soft_limit_enable(true);
	m->config().set_kP(0.1);
	m->config().set_kD(0.07);
	m->config().set_kF(0.060546875);
	m->config().set_motion_cruise_velocity(14500);
	m->config().set_motion_acceleration(22000);
	m->config().set_motion_s_curve_strength(5);
	m->config().apply();
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
	config_climber_motor(left_climber_master);
	config_climber_motor(right_climber_master);

	climber_arm_solenoid = new Solenoid(CLIMBER_MOVEABLE_ARM_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);
	climber_static_hooks_solenoid = new Solenoid(CLIMBER_STATIC_HOOKS_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);


	ros::Rate rate(100);

	while(ros::ok())
	{
		ros::spinOnce();
		step_state_machine();
		publish_diagnostic_data();
		rate.sleep();
	}

	return 0;
}