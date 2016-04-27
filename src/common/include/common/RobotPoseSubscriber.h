/**
 * RobotPoseSubscriber.h
 * Fecha de creaci�n: 24/04/2016, 0:52:08
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Aut�noma de M�xico - UNAM.
 * Instituto de Investigaciones en Matem�ticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-rob�tica.
 * Se�ales Imagenes y Ambientes Virtuales.
 */
#ifndef SRC_ROBOTPOSESUBSCRIBER_H_
#define SRC_ROBOTPOSESUBSCRIBER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

class RobotPoseSubscriber {
public:
	RobotPoseSubscriber(ros::NodeHandle * n);
	virtual ~RobotPoseSubscriber();

	void executeCallback(nav_msgs::Odometry msg);

	geometry_msgs::Pose2D getCurrPos() {
		return currPos;
	}

private:
	geometry_msgs::Pose2D currPos;

protected:
	ros::Subscriber subscriber;
};

#endif /* SRC_ROBOTPOSESUBSCRIBER_H_ */
