/**
 * GotoTaskAction.cpp
 * Fecha de creación: 24/04/2016, 1:11:31
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>

#include <common/RobotPoseSubscriber.h>
#include <common/EnvironmentClient.h>
#include <common/PointClickSubscriber.h>
#include <planning/GoToTaskAction.h>

#include <common/utilMap.h>
#include <common/dijkstra.h>
#include <common/first_depth.h>
#include <common/utilViz.h>
#include <common/voronoiTest.h>

using namespace biorobotics;

class GotoTaskAction {
public:
	GotoTaskAction(std::string name) :
			as(n, name, boost::bind(&GotoTaskAction::executeCallback, this, _1),
					false), action_name(name) {
		as.start();
		env = new EnvironmentClient(&n);
		pose = new RobotPoseSubscriber(&n);
		pointClick = new PointClickSubscriber(&n);
		map_marker_pub = n.advertise<visualization_msgs::Marker>("map_markers",
				10);
		motion_path_pub = n.advertise<nav_msgs::Path>("path_motion", 1);
		polygons_ptr = nullptr;
		num_polygons = 0;
	}
	virtual ~GotoTaskAction() {
		delete env;
		delete pose;
		delete pointClick;
	}
	void executeCallback(const planning::GoToTaskGoalConstPtr msg) {

		Vertex2 * vertexMap;
		int sizeVertexMap = 0;
		bool ** adyacencies;
		int sizeAdyacencies = 0;
		Node pathDijk;

		bool success = true;
		ros::Rate pub_rate(10);

		polygons_ptr = env->convertGeometryMsgToPolygons(env->call(),
				polygons_ptr, &num_polygons);

		Vertex2 init;
		Vertex2 end(msg->goal.x, msg->goal.y);
		if (pointClick->getPoint().x == prevInitPoint.x
				&& pointClick->getPoint().y == prevInitPoint.y)
			init = Vertex2(pose->getCurrPos().x, pose->getCurrPos().y);
		else {
			init = Vertex2(pointClick->getPoint().x, pointClick->getPoint().y);
			prevInitPoint = pointClick->getPoint();
		}

		std::cout << "Execute GotoTaskAction" << std::endl;
		std::cout << "Init (" << init.x << "," << init.y << std::endl;
		std::cout << "End (" << end.x << "," << end.y << std::endl;

		std::vector<biorobotics::Polygon> polygons = std::vector<Polygon>(
				polygons_ptr, polygons_ptr + num_polygons);
		VD * vd = computeVD(polygons);
		vertexMap = createVertexPointerFromVD(vd, &sizeVertexMap);
		adyacencies = computeMapTopologicVD(vd, polygons, 0.3, &sizeAdyacencies,
				vertexMap, sizeVertexMap);
		adyacencies = addNodesInitEndToMap(init, end, polygons.data(),
				polygons.size(), vertexMap, adyacencies, &sizeAdyacencies, 0.3);
		vertexMap = addVertexInitEndToVertexMap(init, end, vertexMap,
				&sizeVertexMap);
		pathDijk = dijsktra(adyacencies, sizeAdyacencies, vertexMap);

		nav_msgs::Path path;
		path.header.frame_id = "map";
		for (int i = 0; i < pathDijk.size; i++) {
			geometry_msgs::PoseStamped poseNode;
			poseNode.pose.position.x = vertexMap[pathDijk.indexPrevious[i]].x;
			poseNode.pose.position.y = vertexMap[pathDijk.indexPrevious[i]].y;
			poseNode.pose.position.z = 0.0;
			poseNode.pose.orientation.w = 1.0;
			poseNode.header.frame_id = "map";
			path.poses.push_back(poseNode);
		}
		geometry_msgs::PoseStamped poseNode;
		poseNode.pose.position.x = vertexMap[pathDijk.index].x;
		poseNode.pose.position.y = vertexMap[pathDijk.index].y;
		poseNode.pose.orientation.w = 1.0;
		poseNode.header.frame_id = "map";
		path.poses.push_back(poseNode);

		motion_path_pub.publish(path);

		int indexGoal = 0;
		float nextGoalX = path.poses[indexGoal].pose.position.x;
		float nextGoalY = path.poses[indexGoal].pose.position.y;

		while (ros::ok()) {

			if (as.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("%s: Preempted", action_name.c_str());
				as.setPreempted();
				success = false;
				break;
			}

			float errorX = nextGoalX - pose->getCurrPos().x;
			float errorY = nextGoalY - pose->getCurrPos().y;
			float error = sqrt(errorX * errorX + errorY * errorY);

			if (error < 0.25) {
				indexGoal++;
				if (indexGoal == path.poses.size())
					break;
				nextGoalX = path.poses[indexGoal].pose.position.x;
				nextGoalY = path.poses[indexGoal].pose.position.y;
			}

			sendToVizMap(vertexMap, adyacencies, sizeAdyacencies,
					&map_marker_pub);
			//sendToVizPath(pathDijk, vertexMap, sizeVertexMap, &map_marker_pub);
			//as.publishFeedback(feedback);

			pub_rate.sleep();
			ros::spinOnce();
		}
		if (success) {
			/*result.position.x = pose->getCurrPos().x;
			 result.position.y = pose->getCurrPos().y;
			 result.position.z = 0.0;*/
			ROS_INFO("%s: Succeeded", action_name.c_str());
			//result.status = std_msgs::Bool(true);
			// set the action state to succeeded
			as.setSucceeded(result);
		}

	}

private:
	ros::NodeHandle n;
	std::string action_name;
	actionlib::SimpleActionServer<planning::GoToTaskAction> as;
	planning::GoToTaskResult result;
	planning::GoToTaskFeedback feedback;
	EnvironmentClient * env;
	RobotPoseSubscriber * pose;
	PointClickSubscriber * pointClick;

	ros::Publisher map_marker_pub;
	ros::Publisher motion_path_pub;
	biorobotics::Polygon * polygons_ptr;
	int num_polygons;
	geometry_msgs::Point32 prevInitPoint;
};

int main(int argc, char ** argv) {

	ros::init(argc, argv, "path_planning");

	GotoTaskAction gotoTask(ros::this_node::getName());

	ros::spin();

}
