/**
 * MotionPotentialFildsCentroids.cpp
 * Fecha de creación: 25/04/2016, 21:24:55
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>

#include <navigation/PotentialFields.h>
#include <common/RobotPoseSubscriber.h>
#include <common/EnvironmentClient.h>
#include <common/definition.h>

float DELTA = 0.09;

int main(int argc, char ** argv) {

	ros::init(argc, argv, "potential_filds_centroids");
	ros::NodeHandle n;
	ros::Rate rate(10);

	if (argc != 3) {
		ROS_INFO("usage: X Y");
		return 1;
	}

	biorobotics::PotentialFields algoPf(10.0, 5.0, 0.5, 2, 5);
	biorobotics::Vertex2 goalPosition;
	goalPosition.x = atof(argv[1]);
	goalPosition.y = atof(argv[2]);

	RobotPoseSubscriber pose(&n);
	EnvironmentClient envcli(&n);
	ros::Publisher motion_goal_pub = n.advertise<geometry_msgs::Pose2D>(
			"goal_motion", 1);

	int num_polygons = 0;
	biorobotics::Polygon * polygons_ptr = nullptr;
	polygons_ptr = envcli.convertGeometryMsgToPolygons(envcli.call(),
			polygons_ptr, &num_polygons);
	std::vector<biorobotics::Polygon> polygons(polygons_ptr,
			polygons_ptr + num_polygons);
	while (ros::ok()) {

		biorobotics::Vertex2 totalForze = algoPf.computeTotalForze(
				biorobotics::Vertex2(pose.getCurrPos().x, pose.getCurrPos().y),
				polygons, goalPosition);

		biorobotics::Vertex2 nextposition;
		nextposition.x = pose.getCurrPos().x
				- DELTA * totalForze.x / totalForze.norm();
		nextposition.y = pose.getCurrPos().y
				- DELTA * totalForze.y / totalForze.norm();

		geometry_msgs::Pose2D pose;
		pose.x = nextposition.x;
		pose.y = nextposition.y;

		motion_goal_pub.publish(pose);

		rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

