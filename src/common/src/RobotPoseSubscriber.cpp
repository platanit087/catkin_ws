/**
 * RobotPoseSubscriber.cpp
 * Fecha de creación: 24/04/2016, 0:52:09
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#include "common/RobotPoseSubscriber.h"

RobotPoseSubscriber::RobotPoseSubscriber(ros::NodeHandle * n) {
	subscriber = n->subscribe("/navigation/localization/base_pose_ground_truth",
			1, &RobotPoseSubscriber::executeCallback, this);
}

RobotPoseSubscriber::~RobotPoseSubscriber() {
	// TODO Auto-generated destructor stub
}

void RobotPoseSubscriber::executeCallback(nav_msgs::Odometry msg) {
	std::cout << "Execute RobotPoseSubscriber" << std::endl;
	//tf::Quaternion quat;
	//tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
	currPos.x = msg.pose.pose.position.x;
	currPos.y = msg.pose.pose.position.y;
	//currPos.theta = quat.getAngle() * quat.getAxis().z();
	currPos.theta = tf::getYaw(msg.pose.pose.orientation);
}

