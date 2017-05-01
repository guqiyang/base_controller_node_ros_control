#include "base_controller.hpp"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"

#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle nh;
	ros::CallbackQueue queue;
	nh.setCallbackQueue(&queue);

	MyRobot robot;
	controller_manager::ControllerManager cm(&robot,nh);

	ros::AsyncSpinner spinner(4, &queue);
	spinner.start();

	ros::Time ts = ros::Time::now();

	ros::Rate rate(50);
	while (ros::ok())
	{
		ros::Duration d = ros::Time::now() - ts;
		ts = ros::Time::now();

		robot.read();
		cm.update(ts, d);
		robot.write();
		robot.writepwm();
		rate.sleep();
	}

	spinner.stop();

	return 0;
}
