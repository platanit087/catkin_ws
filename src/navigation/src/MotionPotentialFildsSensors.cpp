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
#include <sensor_msgs/LaserScan.h>

#include <navigation/PotentialFields.h>
#include <common/RobotPoseSubscriber.h>
#include <common/EnvironmentClient.h>
#include <common/definition.h>

float DELTA = 0.09;

sensor_msgs::LaserScan laser_scan;

void callbackMotionGoal(const sensor_msgs::LaserScan msg) {
	laser_scan = msg;
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "potential_filds_centroids");
	ros::NodeHandle n;
	ros::Rate rate(10);

	if (argc != 3) {
		ROS_INFO("usage: X Y");
		return 1;
	}

	biorobotics::PotentialFields algoPf(2.0, 0.8, 0.5, 2, 5);
	biorobotics::Vertex2 goalPosition;
	goalPosition.x = atof(argv[1]);
	goalPosition.y = atof(argv[2]);

	RobotPoseSubscriber pose(&n);
	EnvironmentClient envcli(&n);
	ros::Publisher motion_goal_pub = n.advertise<geometry_msgs::Pose2D>(
			"goal_motion", 1);
	ros::Subscriber sub_goal_motion = n.subscribe("scan", 1,
			&callbackMotionGoal);

	int num_polygons = 0;
	biorobotics::Polygon * polygons_ptr = nullptr;
	polygons_ptr = envcli.convertGeometryMsgToPolygons(envcli.call(),
			polygons_ptr, &num_polygons);
	std::vector<biorobotics::Polygon> polygons(polygons_ptr,
			polygons_ptr + num_polygons);
	while (ros::ok()) {

		float errorX = pose.getCurrPos().x - goalPosition.x;
		float errorY = pose.getCurrPos().y - goalPosition.y;
		float error = sqrt(errorX * errorX + errorY * errorY);
		if (error < 0.1)
			break;

		biorobotics::Vertex2 totalForze = algoPf.computeTotalForzeWithSensors(
				biorobotics::Vertex2(pose.getCurrPos().x, pose.getCurrPos().y),
				pose.getCurrPos().theta, laser_scan, goalPosition);
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
		/*rate.sleep();
		rate.sleep();
		rate.sleep();
		rate.sleep();
		rate.sleep();*/
		ros::spinOnce();
	}

	return 0;

}

