/**
 * PotentialFields.h
 * Fecha de creaci�n: 22/02/2016, 15:08:54
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Aut�noma de M�xico - UNAM.
 * Instituto de Investigaciones en Matem�ticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-rob�tica.
 * Se�ales Imagenes y Ambientes Virtuales.
 */
#ifndef SRC_SIMULATION_POTENTIALFIELDS_H_
#define SRC_SIMULATION_POTENTIALFIELDS_H_

#include <sensor_msgs/LaserScan.h>
#include <common/utilSimulator.h>

namespace biorobotics {

class PotentialFields {
public:
	PotentialFields(float etha, float dr, float da, float epsilon1,
			float epsilon2);
	virtual ~PotentialFields();
	Vertex2 computeTotalForze(Vertex2 robotPosition,
			std::vector<Polygon> polygons, Vertex2 robotPositionGoal);
	Vertex2 computeTotalForzeWithSensors(Vertex2 robotPosition, float currTheta,
			sensor_msgs::LaserScan laserScan, Vertex2 robotPositionGoal);

protected:
	Vertex2 computeGradientRepulsionObject(Vertex2 robotPosition,
			Vertex2 obstaclePosition, float dr);
	Vertex2 computeRepulsionForze(Vertex2 robotPosition,
			std::vector<Polygon> polygons);
	Vertex2 computeRepulsionForzeWithSensors(Vertex2 robotPosition,
			float currTheta, sensor_msgs::LaserScan laserScan);
	Vertex2 computeAtractionForze(Vertex2 robotPosition,
			Vertex2 robotPositionGoal);

private:
	float etha;
	float dr;
	float da;
	float epsilon1;
	float epsilon2;
};

} /* namespace biorobotics */

#endif /* SRC_SIMULATION_POTENTIALFIELDS_H_ */
