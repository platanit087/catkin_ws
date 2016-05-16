#include <ros/ros.h>

#include "navig_msgs/PathFromAll.h"

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamicvoronoi.h>
#include "path_calculator.h"

costmap_2d::Costmap2DROS * costmap_ros;
costmap_2d::Costmap2D* costmap_;

ros::Publisher voronoi_grid_pub_;
ros::Publisher motion_path_pub;
std::string frame_id_;
DynamicVoronoi voronoi_;
nav_msgs::Path lastCalcPath;

bool callbackDijkstraFromAll(navig_msgs::PathFromAll::Request &req,
		navig_msgs::PathFromAll::Response &resp) {
	ROS_ERROR("On executeCb");
	geometry_msgs::PoseStamped start;
	start.pose.position.x = req.start_pose.position.x;
	start.pose.position.y = req.start_pose.position.y;
	start.pose.position.z = req.start_pose.position.z;
	geometry_msgs::PoseStamped goal;
	goal.pose.position.x = req.goal_pose.position.x;
	goal.pose.position.y = req.goal_pose.position.y;
	goal.pose.position.z = req.goal_pose.position.z;
	costmap_ = costmap_ros->getCostmap();
	std::vector<geometry_msgs::PoseStamped> plan;
	makePlan(costmap_, &voronoi_, start, goal, 0.01, plan);

	ROS_ERROR("Size Plan %d", plan.size());

	lastCalcPath = publishPlan(plan, &motion_path_pub);

	frame_id_ = costmap_ros->getGlobalFrameID();

	publishVoronoiGrid(&voronoi_, costmap_, frame_id_, &voronoi_grid_pub_);

	resp.path = lastCalcPath;

	ROS_ERROR("End executeCb");

	return true;
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "path_calculator_2");
	ros::NodeHandle n;
	ros::ServiceServer srvPathAStarFromAll = n.advertiseService(
			"/navigation/path_planning/path_calculator/dijkstra_from_all",
			callbackDijkstraFromAll);

	tf::TransformListener tf(ros::Duration(10));
	costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf);
	costmap_ros->pause();
	costmap_ros->start();

	voronoi_grid_pub_ = n.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);
	motion_path_pub = n.advertise<nav_msgs::Path>(
			"/navigation/path_planning/path_calculator/last_calc_path", 1);
	ros::Rate rate(10);

	while (ros::ok()) {
		motion_path_pub.publish(lastCalcPath);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;

}
