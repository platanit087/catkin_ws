/**
 * PotentialFields.cpp
 * Fecha de creación: 22/02/2016, 15:08:54
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#include <navigation/PotentialFields.h>
#include <ros/ros.h>

namespace biorobotics {

PotentialFields::PotentialFields(float etha, float dr, float da, float epsilon1,
		float epsilon2) {
	this->etha = etha;
	this->dr = dr;
	this->da = da;
	this->epsilon1 = epsilon1;
	this->epsilon2 = epsilon2;
}

PotentialFields::~PotentialFields() {
	// TODO Auto-generated destructor stub
}

Vertex2 PotentialFields::computeGradientRepulsionObject(Vertex2 robotPosition,
		Vertex2 obstaclePosition, float dr) {
	float module = robotPosition.sub(obstaclePosition).norm();
	float x, y, scalar;
	Vertex2 grad = Vertex2::Zero();
	if (module <= dr) {
		x = robotPosition.x - obstaclePosition.x;
		y = robotPosition.y - obstaclePosition.y;
		scalar = 1 / module - 1 / dr;
		scalar = (-etha * scalar * (1 / pow(module, 3)));
		x = scalar * x;
		y = scalar * y;
		grad.x = x;
		grad.y = y;
	} else {
		grad.x = 0;
		grad.y = 0;
	}
	return grad;
}

Vertex2 PotentialFields::computeRepulsionForze(Vertex2 robotPosition,
		std::vector<Polygon> polygons) {
	std::vector<Vertex2> centroids = computeCentroids(polygons);
	Vertex2 totalForze = Vertex2::Zero();
	for (unsigned int i = 0; i < polygons.size(); i++) {
		Vertex2 forzeObj = computeGradientRepulsionObject(robotPosition,
				centroids.at(i), dr);
		totalForze.x = totalForze.x + forzeObj.x;
		totalForze.y = totalForze.y + forzeObj.y;
	}
	return totalForze;
}

Vertex2 PotentialFields::computeRepulsionForzeWithSensors(Vertex2 robotPosition,
		float currTheta, sensor_msgs::LaserScan laserScan) {
	float inc_angle;
	Vertex2 totalForze = Vertex2::Zero();

	for (int i = 0; i < laserScan.ranges.size(); i++) {
		float range = laserScan.ranges[i];
		float angleSensor = currTheta + i * laserScan.angle_increment
				+ laserScan.angle_min;
		if (range >= laserScan.range_min && range <= laserScan.range_max) {
			Vertex2 obst = Vertex2::Zero();
			float endX = robotPosition.x + range * cos(angleSensor);
			float endy = robotPosition.y + range * sin(angleSensor);
			obst.x = endX;
			obst.y = endy;
			Vertex2 forze = computeGradientRepulsionObject(robotPosition, obst,
					dr);
			totalForze.x = totalForze.x + forze.x;
			totalForze.y = totalForze.y + forze.y;
		}
	}
	return totalForze;
}

Vertex2 PotentialFields::computeAtractionForze(Vertex2 robotPosition,
		Vertex2 robotPositionGoal) {
	float module = robotPosition.sub(robotPositionGoal).norm();
	Vertex2 atractionForze = Vertex2::Zero();
	if (module <= da) {
		atractionForze.x = epsilon1 * (robotPosition.x - robotPositionGoal.x);
		atractionForze.y = epsilon1 * (robotPosition.y - robotPositionGoal.y);
	} else {
		atractionForze.x = epsilon2 / module
				* (robotPosition.x - robotPositionGoal.x);
		atractionForze.y = epsilon2 / module
				* (robotPosition.y - robotPositionGoal.y);
	}
	return atractionForze;
}

Vertex2 PotentialFields::computeTotalForze(Vertex2 robotPosition,
		std::vector<Polygon> polygons, Vertex2 robotPositionGoal) {

	Vertex2 totalForze = Vertex2::Zero();
	Vertex2 atractionForze = computeAtractionForze(robotPosition,
			robotPositionGoal);
	Vertex2 repulsionForze = computeRepulsionForze(robotPosition, polygons);
	totalForze.x = atractionForze.x + repulsionForze.x;
	totalForze.y = atractionForze.y + repulsionForze.y;
	ROS_INFO("atractionForze Fa(%f,%f)", atractionForze.x, atractionForze.y);
	ROS_INFO("repulsionForze Fr(%f,%f)", repulsionForze.x, repulsionForze.y);
	return totalForze;
}

Vertex2 PotentialFields::computeTotalForzeWithSensors(Vertex2 robotPosition,
		float currTheta, sensor_msgs::LaserScan laserScan,
		Vertex2 robotPositionGoal) {
	Vertex2 totalForze = Vertex2::Zero();
	Vertex2 atractionForze = computeAtractionForze(robotPosition,
			robotPositionGoal);
	Vertex2 repulsionForze = computeRepulsionForzeWithSensors(robotPosition,
			currTheta, laserScan);
	totalForze.x = atractionForze.x + repulsionForze.x;
	totalForze.y = atractionForze.y + repulsionForze.y;
	ROS_INFO("atractionForze Fa(%f,%f)", atractionForze.x, atractionForze.y);
	ROS_INFO("repulsionForze Fr(%f,%f)", repulsionForze.x, repulsionForze.y);
	return totalForze;
}

} /* namespace biorobotics */
