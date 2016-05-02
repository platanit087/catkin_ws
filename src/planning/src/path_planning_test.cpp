/**
 * path_planning_test.cpp
 * Fecha de creación: 24/04/2016, 10:08:28
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <planning/GoToTaskAction.h>

typedef actionlib::SimpleActionClient<planning::GoToTaskAction> Client;

int main(int argc, char ** argv) {

	ros::init(argc, argv, "path_planning_test");

	if (argc != 3) {
		ROS_INFO("usage: MotionPlanning xf, yf");
		return 1;
	}

	Client client("path_planning", true);
	client.waitForServer();
	planning::GoToTaskGoal msg;
	msg.goal.x = atof(argv[1]);
	msg.goal.y = atof(argv[2]);
	// Fill in goal here
	client.sendGoal(msg);
	bool finished_before_timeout = client.waitForResult(ros::Duration(110.0));
	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");
	return 0;

}

