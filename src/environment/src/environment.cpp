/**
 * environment.cpp
 * Fecha de creación: 22/04/2016, 14:24:22
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Polygon.h>

#include <common/Environment.h>

#include <common/parserFileWRL.h>
#include <common/triangulateVertex.h>
#include <common/utilViz.h>

biorobotics::Polygon * polygons_ptr;
int num_polygons;

bool environmentServiceCallback(common::Environment::Request &req,
		common::Environment::Response &resp) {
	for (int i = 0; i < num_polygons; i++) {
		biorobotics::Polygon polygon = polygons_ptr[i];
		geometry_msgs::Polygon polygonEnv;
		for (int j = 0; j < polygon.num_vertex; j++) {
			biorobotics::Vertex2 vertex = polygon.vertex[j];
			geometry_msgs::Point32 point;
			point.x = vertex.x;
			point.y = vertex.y;
			point.z = 0.0;
			polygonEnv.points.push_back(point);
		}
		resp.polygons.push_back(polygonEnv);
	}
	return true;
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "environment");
	ros::NodeHandle n;

	std::string path_files_wrl;
	n.getParam("path_files_wrl", path_files_wrl);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(
			"environment_markers", 10);
	ros::ServiceServer motionAngleDistanceService = n.advertiseService(
			"environment_service", environmentServiceCallback);

	std::vector<biorobotics::Polygon> polygons = biorobotics::parserFile(
			path_files_wrl + "bioroboticsmap.wrl");

	polygons_ptr = polygons.data();
	num_polygons = polygons.size();

	std::vector<biorobotics::Triangle> triangulation = biorobotics::traingulate(
			polygons);
	biorobotics::Triangle * triangulationPtr = triangulation.data();
	int num_triangles = triangulation.size();

	std::vector<biorobotics::Triangle> trianglesFloor =
			biorobotics::makeFloorEnvironment(polygons.data(), polygons.size());

	ROS_INFO("size: polygons %d", polygons.size());
	ROS_INFO("size: triangulation %d", triangulation.size());

	ros::Rate loop_rate(5);

	while (ros::ok()) {

		biorobotics::sendToVizEnvironment(triangulationPtr, num_triangles,
				&marker_pub);
		biorobotics::sendToVizFloorEnvironment(trianglesFloor, &marker_pub);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 1;
}
