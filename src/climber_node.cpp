#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include "hmi_agent_node/HMI_Signals.h"

ros::NodeHandle* node;


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
		case ClimberStates::IDLE:
		{
			break;
		}

		case ClimberStates::DEPLOY_INITIAL_HOOKS:
		{
			break;
		}

		case ClimberStates::GRAB_INITIAL_BAR:
		{
			break;
		}

		case ClimberStates::PULL_UP:
		{
			break;
		}

		case ClimberStates::STATIC_LATCH:
		{
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR:
		{
			break;
		}

		case ClimberStates::STATIC_UNLATCH:
		{
			break;
		}

		case ClimberStates::END:
		{
			break;
		}

		case ClimberStates::RETRACT_HOOKS:
		{
			break;
		}

		case ClimberStates::STOPPED:
		{
			break;
		}
	}

	switch(climber_state)
	{
		case ClimberStates::IDLE:
		{
			break;
		}

		case ClimberStates::DEPLOY_INITIAL_HOOKS:
		{
			break;
		}

		case ClimberStates::GRAB_INITIAL_BAR:
		{
			break;
		}

		case ClimberStates::PULL_UP:
		{
			break;
		}

		case ClimberStates::STATIC_LATCH:
		{
			break;
		}

		case ClimberStates::GRAB_NEXT_BAR:
		{
			break;
		}

		case ClimberStates::STATIC_UNLATCH:
		{
			break;
		}

		case ClimberStates::END:
		{
			break;
		}

		case ClimberStates::RETRACT_HOOKS:
		{
			break;
		}

		case ClimberStates::STOPPED:
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

	ros::Rate rate(100);

	while(ros::ok())
	{
		ros::spinOnce();
		step_state_machine();
		rate.sleep();
	}

	return 0;
}