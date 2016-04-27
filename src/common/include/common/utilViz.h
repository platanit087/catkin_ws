/**
 * utilViz.h
 * Fecha de creación: 15/03/2016, 20:06:45
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef INCLUDE_SIMULATOR_SIMULATION_UTILVIZ_H_
#define INCLUDE_SIMULATOR_SIMULATION_UTILVIZ_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <simulator/LaserSensors.h>
#include "utilMap.h"
#include "queue.h"

namespace biorobotics {

void sendToVizLaserSensors(simulator::LaserSensors laserSensors,
		ros::Publisher * marker_pub) {
	visualization_msgs::Marker ray_laser_marker;
	ray_laser_marker.header.frame_id = "/environment";
	ray_laser_marker.header.stamp = ros::Time::now();
	ray_laser_marker.ns = "laser_marker";
	ray_laser_marker.action = visualization_msgs::Marker::ADD;
	ray_laser_marker.pose.orientation.w = 1.0;
	ray_laser_marker.id = 2;
	ray_laser_marker.type = visualization_msgs::Marker::LINE_LIST;
	ray_laser_marker.scale.x = 0.02;
	ray_laser_marker.color.r = 0.0;
	ray_laser_marker.color.g = 0.0;
	ray_laser_marker.color.b = 1.0;
	ray_laser_marker.color.a = 1.0;

	visualization_msgs::Marker end_points_marker;
	end_points_marker.header.frame_id = "/environment";
	end_points_marker.header.stamp = ros::Time::now();
	end_points_marker.ns = "end_points_marker";
	end_points_marker.action = visualization_msgs::Marker::ADD;
	end_points_marker.pose.orientation.w = 1.0;
	end_points_marker.id = 2;
	end_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	end_points_marker.scale.x = 0.03;
	end_points_marker.scale.y = 0.03;
	end_points_marker.scale.z = 0.03;
	end_points_marker.color.r = 1.0;
	end_points_marker.color.g = 1.0;
	end_points_marker.color.b = 1.0;
	end_points_marker.color.a = 1.0;

	for (int i = 0; i < laserSensors.sensors.size(); i++) {
		simulator::Vector3 rayStart = laserSensors.sensors.at(i).rayStart;
		simulator::Vector3 rayEnd = laserSensors.sensors.at(i).rayEnd;
		float distance = laserSensors.sensors.at(i).distance;
		bool instersected = laserSensors.sensors.at(i).instersected;
		geometry_msgs::Point p;
		p.x = rayStart.x;
		p.y = rayStart.y;
		p.z = rayStart.z;
		ray_laser_marker.points.push_back(p);
		end_points_marker.points.push_back(p);
		p.x = rayEnd.x;
		p.y = rayEnd.y;
		p.z = rayEnd.z;
		ray_laser_marker.points.push_back(p);
		end_points_marker.points.push_back(p);
	}
	marker_pub->publish(ray_laser_marker);
	marker_pub->publish(end_points_marker);
}

void sendToVizMap(Vertex2 * vertex, bool ** adyacencies, int sizeAdyacencies,
		ros::Publisher * marker_pub) {

	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/map";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = "map_marker";
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 2;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 0.0;
	map_marker.color.g = 0.0;
	map_marker.color.b = 1.0;
	map_marker.color.a = 1.0;
	for (int i = 0; i < sizeAdyacencies; i++) {
		for (int j = 0; j < sizeAdyacencies; j++) {
			if (adyacencies[i][j]) {
				geometry_msgs::Point p;
				p.x = vertex[i].x;
				p.y = vertex[i].y;
				p.z = 0.0;
				map_marker.points.push_back(p);
				p.x = vertex[j].x;
				p.y = vertex[j].y;
				p.z = 0.0;
				map_marker.points.push_back(p);
			}
		}
	}

	marker_pub->publish(map_marker);
}

void sendToVizInitToEndMap(Vertex2 init, Vertex2 end, Polygon * polygons,
		int sizePolygons, Vertex2 * vertexMap, int sizeVertexMap, float ratio,
		ros::Publisher * marker_pub) {

	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/environment";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = "init_end_marker";
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 3;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 0.0;
	map_marker.color.g = 1.0;
	map_marker.color.b = 0.0;
	map_marker.color.a = 1.0;

	for (int i = 0; i < sizeVertexMap - 2; i++) {
		Vertex2 vertexTargetBind = vertexMap[i];
		Segment sI1;
		Segment sI2;
		Segment sF1;
		Segment sF2;
		Segment sI(init, vertexTargetBind);
		Segment sF(end, vertexTargetBind);
		computeParallelLines(sI, &sI1, &sI2, ratio);
		computeParallelLines(sF, &sF1, &sF2, ratio);

		geometry_msgs::Point p;

		p.x = sI1.v1.x;
		p.y = sI1.v1.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sI1.v2.x;
		p.y = sI1.v2.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sI2.v1.x;
		p.y = sI2.v1.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sI2.v2.x;
		p.y = sI2.v2.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sF1.v1.x;
		p.y = sF1.v1.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sF1.v2.x;
		p.y = sF1.v2.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sF2.v1.x;
		p.y = sF2.v1.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = sF2.v2.x;
		p.y = sF2.v2.y;
		p.z = 0.0;
		map_marker.points.push_back(p);

		p.x = init.x;
		p.y = init.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = vertexTargetBind.x;
		p.y = vertexTargetBind.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = end.x;
		p.y = end.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = vertexTargetBind.x;
		p.y = vertexTargetBind.y;
		p.z = 0.0;
		map_marker.points.push_back(p);

	}
	marker_pub->publish(map_marker);
}

void sendToVizBadInitToEndMap(Vertex2 init, Vertex2 end, Polygon * polygons,
		int sizePolygons, Vertex2 * vertexMap, int sizeVertexMap, float ratio,
		ros::Publisher * marker_pub) {

	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/environment";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = "bad_init_end_marker";
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 3;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 0.0;
	map_marker.color.g = 1.0;
	map_marker.color.b = 0.0;
	map_marker.color.a = 1.0;

	for (int i = 0; i < sizeVertexMap - 2; i++) {
		Vertex2 vertexTargetBind = vertexMap[i];
		Segment sI1;
		Segment sI2;
		Segment sF1;
		Segment sF2;
		Segment sI(init, vertexTargetBind);
		Segment sF(end, vertexTargetBind);
		computeParallelLines(sI, &sI1, &sI2, ratio);
		computeParallelLines(sF, &sF1, &sF2, ratio);

		geometry_msgs::Point p;

		if (isnan(sI1.v1.x) || isnan(sI1.v1.y) || isnan(sI2.v1.x)
				|| isnan(sI2.v1.y)) {
			p.x = init.x;
			p.y = init.y;
			p.z = 0.0;
			map_marker.points.push_back(p);
			p.x = vertexTargetBind.x;
			p.y = vertexTargetBind.y;
			p.z = 0.0;
			map_marker.points.push_back(p);
		}
		if (isnan(sF1.v1.x) || isnan(sF1.v1.y) || isnan(sF2.v1.x)
				|| isnan(sF2.v1.y)) {
			p.x = end.x;
			p.y = end.y;
			p.z = 0.0;
			map_marker.points.push_back(p);
			p.x = vertexTargetBind.x;
			p.y = vertexTargetBind.y;
			p.z = 0.0;
			map_marker.points.push_back(p);
		}

	}
	marker_pub->publish(map_marker);
}

void sendToVizPath(Node path, Vertex2 * vertexMap, int sizeVertexMap,
		ros::Publisher * marker_pub) {
	visualization_msgs::Marker path_marker;
	path_marker.header.frame_id = "/map";
	path_marker.header.stamp = ros::Time::now();
	path_marker.ns = "path_marker";
	path_marker.action = visualization_msgs::Marker::ADD;
	path_marker.pose.orientation.w = 1.0;
	path_marker.id = 3;
	path_marker.type = visualization_msgs::Marker::LINE_LIST;
	path_marker.scale.x = 0.02;
	path_marker.color.r = 1.0;
	path_marker.color.g = 1.0;
	path_marker.color.b = 0.0;
	path_marker.color.a = 1.0;

	geometry_msgs::Point p;
	for (int i = 0; i < path.size - 1; i++) {
		p.x = vertexMap[path.indexPrevious[i]].x;
		p.y = vertexMap[path.indexPrevious[i]].y;
		p.z = 0.0;
		path_marker.points.push_back(p);
		p.x = vertexMap[path.indexPrevious[i + 1]].x;
		p.y = vertexMap[path.indexPrevious[i + 1]].y;
		p.z = 0.0;
		path_marker.points.push_back(p);
	}
	p.x = vertexMap[path.indexPrevious[path.size - 1]].x;
	p.y = vertexMap[path.indexPrevious[path.size - 1]].y;
	p.z = 0.0;
	path_marker.points.push_back(p);
	p.x = vertexMap[path.index].x;
	p.y = vertexMap[path.index].y;
	p.z = 0.0;
	path_marker.points.push_back(p);
	marker_pub->publish(path_marker);
}

void sendToVizFloorEnvironment(
		std::vector<biorobotics::Triangle> trianglesFloor,
		ros::Publisher * marker_pub) {
	visualization_msgs::Marker floor_marker;
	floor_marker.header.frame_id = "/map";
	floor_marker.header.stamp = ros::Time::now();
	floor_marker.ns = "floor_marker";
	floor_marker.action = visualization_msgs::Marker::ADD;
	floor_marker.pose.orientation.w = 1.0;
	floor_marker.id = 3;
	floor_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	floor_marker.scale.x = 1.0;
	floor_marker.scale.y = 1.0;
	floor_marker.scale.z = 1.0;
	floor_marker.color.r = 0.478f;
	floor_marker.color.g = 0.478f;
	floor_marker.color.b = 0.478f;
	floor_marker.color.a = 1.0;

	for (int i = 0; i < trianglesFloor.size(); i++) {
		biorobotics::Triangle triangle = trianglesFloor[i];
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		geometry_msgs::Point p3;
		p1.x = triangle.v1.x;
		p1.y = triangle.v1.y;
		p1.z = triangle.v1.z;
		p2.x = triangle.v2.x;
		p2.y = triangle.v2.y;
		p2.z = triangle.v2.z;
		p3.x = triangle.v3.x;
		p3.y = triangle.v3.y;
		p3.z = triangle.v3.z;

		floor_marker.points.push_back(p1);
		floor_marker.points.push_back(p2);
		floor_marker.points.push_back(p3);
	}

	marker_pub->publish(floor_marker);

}

void sendToVizEnvironment(biorobotics::Triangle * triangulationPtr,
		int num_triangles, ros::Publisher * marker_pub) {

	/*Marker Environment*/
	visualization_msgs::Marker objects_marker;
	objects_marker.header.frame_id = "/map";
	objects_marker.header.stamp = ros::Time::now();
	objects_marker.ns = "objects_marker";
	objects_marker.action = visualization_msgs::Marker::ADD;
	objects_marker.pose.position.z = 0.0;
	objects_marker.pose.orientation.w = 1.0;
	objects_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	objects_marker.scale.x = 1.0;
	objects_marker.scale.y = 1.0;
	objects_marker.scale.z = 1.0;
	objects_marker.color.r = 0.6f;
	objects_marker.color.g = 0.4f;
	objects_marker.color.b = 0.2f;
	objects_marker.color.a = 1.0;

	/*Marker Environment*/
	visualization_msgs::Marker walls_marker;
	walls_marker.header.frame_id = "/map";
	walls_marker.header.stamp = ros::Time::now();
	walls_marker.ns = "walls_marker";
	walls_marker.action = visualization_msgs::Marker::ADD;
	walls_marker.pose.position.z = 0.0;
	walls_marker.pose.orientation.w = 1.0;
	walls_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	walls_marker.scale.x = 1.0;
	walls_marker.scale.y = 1.0;
	walls_marker.scale.z = 1.0;
	walls_marker.color.r = 0.478f;
	walls_marker.color.g = 0.478f;
	walls_marker.color.b = 0.478f;
	walls_marker.color.a = 1.0;

	for (int i = 0; i < num_triangles; i++) {
		biorobotics::Triangle triangle = triangulationPtr[i];

		geometry_msgs::Point p1;
		p1.x = triangle.v1.x;
		p1.y = triangle.v1.y;
		p1.z = triangle.v1.z;

		geometry_msgs::Point p2;
		p2.x = triangle.v2.x;
		p2.y = triangle.v2.y;
		p2.z = triangle.v2.z;

		geometry_msgs::Point p3;
		p3.x = triangle.v3.x;
		p3.y = triangle.v3.y;
		p3.z = triangle.v3.z;

		if (triangle.objectType == OBSTACLE) {
			objects_marker.points.push_back(p1);
			objects_marker.points.push_back(p2);
			objects_marker.points.push_back(p3);
		} else {
			walls_marker.points.push_back(p1);
			walls_marker.points.push_back(p2);
			walls_marker.points.push_back(p3);
		}
	}
	if (objects_marker.points.size() > 0)
		marker_pub->publish(objects_marker);
	if (walls_marker.points.size() > 0)
		marker_pub->publish(walls_marker);
}

}

#endif /* INCLUDE_SIMULATOR_SIMULATION_UTILVIZ_H_ */
