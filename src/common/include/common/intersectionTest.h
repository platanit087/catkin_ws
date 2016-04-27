/**
 * intersectionTest.h
 * Fecha de creación: 12/02/2016, 10:21:08
 *
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#ifndef INCLUDE_SIMULATOR_INTERSECTIONTEST_H_
#define INCLUDE_SIMULATOR_INTERSECTIONTEST_H_

#include "definition.h"
#include "utilSimulator.h"
#include <vector>
#include <math.h>

#define EPSILON 0.000001

#define AXISTEST_X01(a, b, fa, fb) p0 = a*v0(1,0) - b*v0(2,0); p2 = a*v2(1,0) - b*v2(2,0); if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} rad = fa * boxhalfsize(1,0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;
#define AXISTEST_X2(a, b, fa, fb) p0 = a*v0(1,0) - b*v0(2,0); p1 = a*v1(1,0) - b*v1(2,0); if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} rad = fa * boxhalfsize(1,0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;

#define AXISTEST_Y02(a, b, fa, fb) p0 = -a*v0(0,0) + b*v0(2,0); p2 = -a*v2(0,0) + b*v2(2,0); if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;}rad = fa * boxhalfsize(0,0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;
#define AXISTEST_Y1(a, b, fa, fb) p0 = -a*v0(0, 0) + b*v0(2,0); p1 = -a*v1(0, 0) + b*v1(2,0); if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} rad = fa * boxhalfsize(0, 0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;

#define AXISTEST_Z12(a, b, fa, fb) p1 = a*v1(0,0) - b*v1(1,0); p2 = a*v2(0,0) - b*v2(1,0); if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} rad = fa * boxhalfsize(0,0) + fb * boxhalfsize(1,0); if(min>rad || max<-rad) return 0;
#define AXISTEST_Z0(a, b, fa, fb) p0 = a*v0(0,0) - b*v0(1,0); p1 = a*v1(0,0) - b*v1(1,0); if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} rad = fa * boxhalfsize(0,0) + fb * boxhalfsize(1,0); if(min>rad || max<-rad) return 0;

#define FINDMINMAX(x0,x1,x2,min,max) min = max = x0; if(x1<min) min=x1; if(x1>max) max=x1; if(x2<min) min=x2; if(x2>max) max=x2;

namespace biorobotics {

/*bool planeBoxOverlap(Eigen::Vector3d normal, Eigen::Vector3d vert,
 Eigen::Vector3d maxbox) {

 int q;

 Eigen::Vector3d vmin, vmax;
 float v;

 for (q = 0; q <= 2; q++) {

 v = vert(q, 0);

 if (normal(q, 0) > 0.0f) {
 vmin(q, 0) = -maxbox(q, 0) - v;
 vmax(q, 0) = maxbox(q, 0) - v;
 } else {
 vmin(q, 0) = maxbox(q, 0) - v;
 vmax(q, 0) = -maxbox(q, 0) - v;
 }
 }

 if (normal.dot(vmin) > 0.0f)
 return false;

 if (normal.dot(vmax) >= 0.0f)
 return true;

 return false;

 }*/

/*bool triBoxOverlap(Eigen::Vector3d boxcenter, Eigen::Vector3d boxhalfsize,
 biorobotics::Triangle triverts) {

 Eigen::Vector3d v0, v1, v2;
 Eigen::Vector3d e0, e1, e2;
 Eigen::Vector3d normal;

 float min, max, p0, p1, p2, rad, fex, fey, fez;

 v0 = triverts.v1 - boxcenter;
 v1 = triverts.v2 - boxcenter;
 v2 = triverts.v3 - boxcenter;

 e0 = v1 - v0;
 e1 = v2 - v1;
 e2 = v0 - v2;

 fex = fabs((double) e0(0, 0));
 fey = fabs((double) e0(1, 0));
 fez = fabs((double) e0(2, 0));

 AXISTEST_X01(e0(2, 0), e0(1, 0), fez, fey);
 AXISTEST_Y02(e0(2, 0), e0(0, 0), fez, fex);
 AXISTEST_Z12(e0(1, 0), e0(0, 0), fey, fex);

 fex = fabs((double) e1(0, 0));
 fey = fabs((double) e1(1, 0));
 fez = fabs((double) e1(2, 0));

 AXISTEST_X01(e1(2, 0), e1(1, 0), fez, fey);
 AXISTEST_Y02(e1(2, 0), e1(0, 0), fez, fex);
 AXISTEST_Z0(e1(1, 0), e1(0, 0), fey, fex);

 fex = fabs((double) e2(0, 0));
 fey = fabs((double) e2(1, 0));
 fez = fabs((double) e2(2, 0));

 AXISTEST_X2(e2(2, 0), e2(1, 0), fez, fey);
 AXISTEST_Y1(e2(2, 0), e2(0, 0), fez, fex);
 AXISTEST_Z12(e2(1, 0), e2(0, 0), fey, fex);

 FINDMINMAX(v0(0, 0), v1(0, 0), v2(0, 0), min, max);
 if (min > boxhalfsize(0, 0) || max < -boxhalfsize(0, 0))
 return false;

 FINDMINMAX(v0(1, 0), v1(1, 0), v2(1, 0), min, max);
 if (min > boxhalfsize(1, 0) || max < -boxhalfsize(1, 0))
 return false;

 FINDMINMAX(v0(2, 0), v1(2, 0), v2(2, 0), min, max);
 if (min > boxhalfsize(2, 0) || max < -boxhalfsize(2, 0))
 return false;

 normal = e0.cross(e1);

 if (!planeBoxOverlap(normal, v0, boxhalfsize))
 return false;

 return true;

 }*/

/*bool testCollisionTriangleBox(std::vector<biorobotics::Triangle> triangles,
 biorobotics::Box box, Eigen::Matrix4d matrixBox) {

 bool test = false;
 Eigen::Matrix3d rotation = matrixBox.block<3, 3>(0, 0);

 std::vector<biorobotics::Triangle> out;

 for (int i = 0; i < triangles.size() && !test; i++) {
 biorobotics::Triangle triangle = triangles.at(i);

 Eigen::Vector3d vt1 = Eigen::Vector3d::Ones();
 Eigen::Vector3d vt2 = Eigen::Vector3d::Ones();
 Eigen::Vector3d vt3 = Eigen::Vector3d::Ones();

 vt1 = triangle.v1;
 vt2 = triangle.v2;
 vt3 = triangle.v3;

 vt1 = rotation.inverse() * vt1;
 vt2 = rotation.inverse() * vt2;
 vt3 = rotation.inverse() * vt3;

 out.push_back(
 biorobotics::Triangle(vt1.head<3>(), vt2.head<3>(),
 vt3.head<3>(), triangle.objectType));

 biorobotics::Triangle triangleOritented = biorobotics::Triangle(
 vt1.head<3>(), vt2.head<3>(), vt3.head<3>(),
 triangle.objectType);

 test = triBoxOverlap(box.center, box.boxhalfsize, triangleOritented);
 }
 //std::cout << "test:" << test << std::endl;
 return test;

 }*/

/*bool RayTriangleIntersect(Eigen::Vector3d orig, Eigen::Vector3d dir,
 Eigen::Vector3d vert0, Eigen::Vector3d vert1, Eigen::Vector3d vert2,
 float *t, float *u, float *v) {
 Eigen::Vector3d edge1, edge2, tvec, pvec, qvec;
 float det, inv_det;

 edge1 = vert1 - vert0;
 edge2 = vert2 - vert0;

 pvec = dir.cross(edge2);

 det = edge1.dot(pvec);

 if (det < EPSILON)
 return false;

 tvec = orig - vert0;

 *u = tvec.dot(pvec);
 if (*u < 0.0 || *u > det)
 return false;

 qvec = tvec.cross(edge1);

 *v = dir.dot(qvec);
 if (*v < 0.0 || *u + *v > det)
 return false;

 *t = edge2.dot(qvec);
 inv_det = 1.0 / det;
 *t *= inv_det;
 *u *= inv_det;
 *v *= inv_det;

 if (det > -EPSILON && det < EPSILON)
 return false;
 inv_det = 1.0 / det;

 tvec = orig - vert0;

 *u = tvec.dot(pvec) * inv_det;
 if (*u < 0.0 || *u > 1.0)
 return false;

 qvec = tvec.cross(edge1);

 *v = dir.dot(qvec) * inv_det;
 if (*v < 0.0 || *u + *v > 1.0)
 return false;

 *t = edge2.dot(qvec) * inv_det;
 return true;
 }*/

bool testSegmentIntersect(Segment segment1, Segment segment2) {
	float determinante1 = getDeterminant(segment1.v2, segment1.v1, segment2.v1);
	float determinante2 = getDeterminant(segment1.v2, segment1.v1, segment2.v2);
	if ((determinante1 < 0 && determinante2 > 0)
			|| (determinante1 > 0 && determinante2 < 0)) {
		determinante1 = getDeterminant(segment2.v2, segment2.v1, segment1.v1);
		determinante2 = getDeterminant(segment2.v2, segment2.v1, segment1.v2);
		if ((determinante1 < 0 && determinante2 > 0)
				|| (determinante1 > 0 && determinante2 < 0))
			return true;
		else
			return false;
	} else if (determinante1 == 0 || determinante2 == 0)
		return true;
	else
		return false;
}

Vertex2 computeIntersection(Segment s1, Segment s2) {
	//float A1 = s1.getPoint2().getY() - s1.getPoint1().getY();
	float A1 = s1.v2.y - s1.v1.y;
	//float B1 = s1.getPoint1().getX() - s1.getPoint2().getX();
	float B1 = s1.v1.x - s1.v2.x;
	//float C1 = A1 * s1.getPoint1().getX() + B1 * s1.getPoint1().getY();
	float C1 = A1 * s1.v1.x + B1 * s1.v1.y;
	//float A2 = s2.getPoint2().getY() - s2.getPoint1().getY();
	float A2 = s2.v2.y - s2.v1.y;
	//float B2 = s2.getPoint1().getX() - s2.getPoint2().getX();
	float B2 = s2.v1.x - s2.v2.x;
	//float C2 = A2 * s2.getPoint1().getX() + B2 * s2.getPoint1().getY();
	float C2 = A2 * s2.v1.x + B2 * s2.v1.y;
	double det = A1 * B2 - A2 * B1;
	double x = (B2 * C1 - B1 * C2) / det;
	double y = (A1 * C2 - A2 * C1) / det;

	return Vertex2(x, y);
}

/*float computeDistanceEuclideanPointToLine(Eigen::Vector3d point,
 Segment segment) {
 Eigen::Vector3d u;
 Eigen::Vector3d p0;
 u(0, 0) = segment.v2.x - segment.v1.x;
 u(1, 0) = segment.v2.y - segment.v1.y;
 u(2, 0) = 0.0;
 p0(0, 0) = segment.v1.x;
 p0(1, 0) = segment.v1.y;
 p0(2, 0) = 0.0;
 Eigen::Vector3d ba = point - p0;
 return ba.cross(u).norm() / u.norm();
 }*/

bool isInRangeProyection(Vertex2 vertex, Segment segment) {
	float xinf, xsup, yinf, ysup;
	if (segment.v1.x < segment.v2.x) {
		xinf = segment.v1.x;
		xsup = segment.v2.x;
	} else {
		xsup = segment.v1.x;
		xinf = segment.v2.x;
	}
	if (segment.v1.y < segment.v2.y) {
		yinf = segment.v1.y;
		ysup = segment.v2.y;
	} else {
		ysup = segment.v1.y;
		yinf = segment.v2.y;
	}
	if (vertex.x >= xinf && vertex.x <= xsup && vertex.y >= yinf
			&& vertex.y <= ysup)
		return true;
	else
		return false;
}

bool SegmentCircleIntersect(Circle circle, Segment segment) {
	bool result = false;
	float dx = segment.v2.x - segment.v1.x;
	float dy = segment.v2.y - segment.v1.y;
	float h = circle.center.x;
	float k = circle.center.y;
	float r = circle.ratio;
	float x0 = segment.v1.x;
	float y0 = segment.v1.y;

	float a = pow(dx, 2) + pow(dy, 2);
	float b = 2 * dx * (x0 - h) + 2 * dy * (y0 - k);
	float c = pow(x0 - (float) h, 2) + pow(y0 - (float) k, 2) - pow(r, 2);

	float disc = pow(b, 2) - 4 * a * c;

	if (disc == 0 || disc > 0) {
		float t1 = (-b + sqrt(disc)) / (2 * a);
		float t2 = (-b - sqrt(disc)) / (2 * a);

		float x1int = dx * t1 + x0;
		float y1int = dy * t1 + y0;
		float x2int = dx * t2 + x0;
		float y2int = dy * t2 + y0;

		if (disc == 0) {
			//std::cout << "Tangent" << std::endl;
			result = isInRangeProyection(Vertex2(x1int, y1int), segment);
		} else if (disc > 0) {
			//std::cout << "Intersection" << std::endl;
			result = isInRangeProyection(Vertex2(x1int, y1int), segment);
			if (!result)
				result = isInRangeProyection(Vertex2(x2int, y2int), segment);
		}
	} else if (disc < 0) {
		//std::cout << "Not intersect" << std::endl;
	}
	return result;
}

bool testCircleWithPolygons(float posx, float posy, float radius,
		Polygon * polygons, int num_polygons) {

	bool testcollision = false;
	Circle shapeRobot(Vertex2(posx, posy), radius);

	for (int i = 0; i < num_polygons && !testcollision; i++) {
		Polygon polygon = polygons[i];
		for (int j = 0; j < polygon.num_vertex && !testcollision; j++) {
			if (j < polygon.num_vertex - 1) {
				testcollision = SegmentCircleIntersect(shapeRobot,
						Segment(
								Segment(polygon.vertex[j],
										polygon.vertex[j + 1])));
			} else {
				testcollision = SegmentCircleIntersect(shapeRobot,
						Segment(Segment(polygon.vertex[j], polygon.vertex[0])));
			}
		}
	}

	return testcollision;
}

}

#endif /* INCLUDE_SIMULATOR_INTERSECTIONTEST_H_ */
