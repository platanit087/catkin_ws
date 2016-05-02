/**
 * basic_motion.cpp
 * Fecha de creación: 24/04/2016, 13:36:20
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>

#include <navigation/LowLevelControl.h>
#include <common/RobotPoseSubscriber.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>

bool newPath = false;
bool newGoal = false;
nav_msgs::PathConstPtr path_ptr;
int curr_index_nav;
geometry_msgs::Pose2DConstPtr goal_ptr;

LowLevelControl control;

void callbackMotionGoal(const geometry_msgs::Pose2DConstPtr msg) {
	goal_ptr = msg;
	newGoal = true;
}

void callbackMotionPath(const nav_msgs::PathConstPtr msg) {
	newPath = true;
	curr_index_nav = 0;
	path_ptr = msg;
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "path_motion");
	ros::NodeHandle n;

	ros::Publisher motionPub = n.advertise<std_msgs::Float32MultiArray>(
			"/hardware/mobile_base/speeds", 1);
	ros::Subscriber sub_path_motion = n.subscribe("path_motion", 1,
			&callbackMotionPath);
	ros::Subscriber sub_goal_motion = n.subscribe("goal_motion", 1,
			&callbackMotionGoal);
	ros::Rate rate(10);

	RobotPoseSubscriber pose(&n);

	std_msgs::Float32MultiArray speeds;
	speeds.data.push_back(0);
	speeds.data.push_back(0);

	control.SetRobotParams(0.48);

	while (ros::ok()) {

		float xpos = pose.getCurrPos().x;
		float ypos = pose.getCurrPos().y;
		float anglepos = pose.getCurrPos().theta;

		if (newPath) {
			float goalx = path_ptr->poses[curr_index_nav].pose.position.x;
			float goaly = path_ptr->poses[curr_index_nav].pose.position.y;
			float errorX = goalx - xpos;
			float errorY = goaly - ypos;
			float error = sqrt(errorX * errorX + errorY * errorY);
			if (error < 0.1) {
				if (curr_index_nav < path_ptr->poses.size() - 1)
					curr_index_nav++;
				else {
					speeds.data[0] = 0;
					speeds.data[1] = 0;
					motionPub.publish(speeds);
					newPath = false;
				}
			} else {
				control.CalculateSpeeds(xpos, ypos, anglepos, goalx, goaly,
						speeds.data[0], speeds.data[1], false);
				motionPub.publish(speeds);
			}
		}
		if (newGoal) {
			float errorX = goal_ptr->x - xpos;
			float errorY = goal_ptr->y - ypos;
			float error = sqrt(errorX * errorX + errorY * errorY);
			if (error < 0.05) {
				speeds.data[0] = 0;
				speeds.data[1] = 0;
				motionPub.publish(speeds);
				newGoal = false;
			} else {
				control.CalculateSpeeds(xpos, ypos, anglepos, goal_ptr->x,
						goal_ptr->y, speeds.data[0], speeds.data[1], false);
				motionPub.publish(speeds);
			}
		}

		rate.sleep();
		ros::spinOnce();
	}

}
