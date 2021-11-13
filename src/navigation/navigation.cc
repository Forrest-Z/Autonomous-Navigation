
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <algorithm>
#include "simple_queue.h"
#include <unordered_map>
using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using visualization::DrawPathOption;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::ClearVisualizationMsg;
using visualization::DrawLine;
using visualization::DrawCross;
using geometry::line2f;

using namespace math_util;
using namespace ros_helpers;

namespace {
	ros::Publisher drive_pub_;
	ros::Publisher viz_pub_;
	VisualizationMsg local_viz_msg_;
	VisualizationMsg global_viz_msg_;
	AckermannCurvatureDriveMsg drive_msg_;
	// Epsilon value for handling limited numerical precision.
	const float kEpsilon = 1e-5;
} //namespace

namespace navigation {


	Navigation::Navigation(const std::string& map_file, ros::NodeHandle* n) :
		robot_loc_(0, 0),
		robot_angle_(0),
		robot_vel_(0, 0),
		robot_omega_(0),
		nav_complete_(false),
		nav_goal_loc_(0, 0),
		nav_goal_angle_(0) {
		drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
			"ackermann_curvature_drive", 1);
		viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
		local_viz_msg_ = visualization::NewVisualizationMessage(
			"base_link", "navigation_local");
		global_viz_msg_ = visualization::NewVisualizationMessage(
			"map", "navigation_global");
		InitRosHeader("base_link", &drive_msg_.header);
	}

	void Navigation::InitializeMap(const std::string& map_file) {
		map_.Load("maps/" + map_file + ".txt");
		printf("Initializing map!\n");
	}

	void Navigation::neighbors(const Vector2f vertix, vector<int>& neighbour_) {
		std::vector<Eigen::Vector2f> dirs{ {grid_resolution_,0} , {0,grid_resolution_}, {-grid_resolution_,0}, {0,-grid_resolution_},
			{grid_resolution_,grid_resolution_}, {grid_resolution_,-grid_resolution_}, {-grid_resolution_,grid_resolution_}, {-grid_resolution_,-grid_resolution_} };

		for (size_t i = 0; i < dirs.size(); i++) {
			Vector2f neighbor(vertix.x() + dirs[i].x(), vertix.y() + dirs[i].y());
			for (size_t n = 0; n < vertices.size(); n++) {
				if (neighbor == vertices[n]) {
					neighbour_.push_back(n);
					break;
				}
			}
			/*if (abs(neighbor.x()) <= 50 && abs(neighbor.y()) <= 50)
			neighbour_.push_back(neighbor);*/
		}
	}

	void Navigation::createGrid() {
		//Populate Uniform Grid 
		printf("Creating Grid ... ");
		for (float x = -50; x <= 50; x += grid_resolution_)
			for (float y = -50; y <= 50; y += grid_resolution_)
				vertices.push_back(Vector2f(x, y));
		printf("Done\n");
		
		printf("Adding Edges ... ");
		//Add edges 
		std::vector<int> neighbours_id_;
		for (size_t i = 0; i < vertices.size(); i++) {
			neighbours_id_.clear();
			neighbors(vertices[i], neighbours_id_);
			edges.push_back(neighbours_id_);
		}
		printf("Done\n");
		
		//Line segments correspond to obstacles in the world, graph of uniform grid
		//removing those edges which collide with the map
		printf("Removing Map Intersections ... ");
		bool intersects;
		Vector2f intersection_point;
		neighbours_id_.clear();
		neighbours_id_.clear();
		for (size_t v = 0; v < vertices.size(); v++) {
			neighbours_id_ = edges.at(v);
			for (size_t n = 0; n < neighbours_id_.size(); n++) {
				line2f my_line(vertices[v].x(), vertices[v].y(), vertices[neighbours_id_[n]].x(), vertices[neighbours_id_[n]].y());
				line2f virtual_line_up(vertices[v].x(), vertices[v].y()+safe_distance_, vertices[neighbours_id_[n]].x(), vertices[neighbours_id_[n]].y()+safe_distance_);
				line2f virtual_line_down(vertices[v].x(), vertices[v].y()-safe_distance_, vertices[neighbours_id_[n]].x(), vertices[neighbours_id_[n]].y()-safe_distance_);
				line2f virtual_line_left(vertices[v].x()-safe_distance_, vertices[v].y(), vertices[neighbours_id_[n]].x()-safe_distance_, vertices[neighbours_id_[n]].y());
				line2f virtual_line_right(vertices[v].x()+safe_distance_, vertices[v].y(), vertices[neighbours_id_[n]].x()+safe_distance_, vertices[neighbours_id_[n]].y());
				for (size_t i = 0; i < map_.lines.size(); i++) {
					const line2f map_line = map_.lines[i];

					intersects = map_line.Intersection(my_line, &intersection_point);
					bool intersects_up = map_line.Intersection(virtual_line_up, &intersection_point);
					bool intersects_down = map_line.Intersection(virtual_line_down, &intersection_point);
					bool intersects_left = map_line.Intersection(virtual_line_left, &intersection_point);
					bool intersects_right = map_line.Intersection(virtual_line_right, &intersection_point);

					if (intersects || intersects_up || intersects_down || intersects_left || intersects_right) {
						//remove the edge
						edges[v].erase(std::remove(edges[v].begin(), edges[v].end(), neighbours_id_[n]), edges[v].end());
						edges[neighbours_id_[n]].erase(std::remove(edges[neighbours_id_[n]].begin(), edges[neighbours_id_[n]].end(), v), edges[neighbours_id_[n]].end());
						//printf("removed %f %f from vertix %f %f\n", vertices[neighbours_id_[n]].x(), vertices[neighbours_id_[n]].y(), vertices[v].x(), vertices[v].y());
					}
				}
			}
			neighbours_id_.clear();
		}
		printf("Done\n");
	}

	double Navigation::Heuristic(Vector2f& a, Vector2f& b) {
		//return abs(a.x() - b.x()) * 10 + abs(a.y() - b.y()) * 10;
		//return abs(a.x() - b.x()) * 1.05 + abs(a.y() - b.y()) * 1.05;
		return (a-b).norm();
	}

	int Navigation::SearchNearestPoint(Vector2f pos) {
		double smallest_man_dis = 3000.0;
		int closest_point_id = -1;
		for (size_t v = 0; v < vertices.size(); v++) {
			if (!edges.at(v).empty()) {
				double man_dis = Heuristic(pos, vertices[v]);
				if (man_dis < smallest_man_dis) {
					closest_point_id = v;
					smallest_man_dis = man_dis;
				}
			}
		}
		return closest_point_id;
	}

	void Navigation::AStarSearch() {
		std::unordered_map<int, double> cost;
		SimpleQueue<int, double> frontier;
		start_id_ = SearchNearestPoint(odom_loc_);
		goal_id_ = SearchNearestPoint(nav_goal_loc_);

		frontier.Push(start_id_, 0);
		path_parent_[start_id_] = start_id_;
		cost[start_id_] = 0;

		while (!frontier.Empty()) {
			int current = frontier.Pop();

			if (current == goal_id_) { break; }

			for (int next : edges.at(current)) {
				float new_cost = cost[current] + Heuristic(vertices[current], vertices[next]);
				if ((cost.find(next) == cost.end()) || (new_cost < cost[next])) {
					cost[next] = new_cost;
					double priority = new_cost + Heuristic(vertices[next], vertices[goal_id_]) * 1.1;
					frontier.Push(next, priority);
					path_parent_[next] = current;
				}
			}
		}
	}

	void Navigation::ReconstructPath() {
		path_.clear();
		int start_id = SearchNearestPoint(odom_loc_);
		int current = SearchNearestPoint(nav_goal_loc_);

		while (current != start_id) {
			path_.push_back(current);
			current = path_parent_[current];
		}
		path_.push_back(start_id);
		std::reverse(path_.begin(), path_.end());
		curr_path_ndx_ = 0;
		path_is_ready_ = true;
	}

	void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
		printf("Setting Nav Goal to %f %f\n", loc[0], loc[1]);
		nav_goal_loc_ = loc;
		nav_target_set_ = true;
		path_is_ready_ = false;
		path_parent_.clear();
		AStarSearch();
		ReconstructPath();
	}

	void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
		start_loc_ = loc;
	}

	void Navigation::UpdateOdometry(const Vector2f& loc,
		float angle,
		const Vector2f& vel,
		float ang_vel) {

		if (!is_odom_init_) // First time odometry is available
		{
			initial_odom_loc_ = loc;
			initial_odom_angle_ = angle;
			is_odom_init_ = true;
		}

		odom_loc_ = loc;
		odom_angle_ = angle;
		robot_vel_ = vel;
		robot_omega_ = ang_vel;
	}

	void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
		double time) {


		for (Vector2f point : cloud) {
			DrawPoint(point, 0xe62020, local_viz_msg_);
		}
		if (!nav_complete_ && path_is_ready_) {
			FindBestPath(cloud);
		}
	}

	void Navigation::TOC() {
		float obs_vel = robot_vel_.norm();

		dist_to_stop_ = Sq(obs_vel) / (2.0 * max_decel_) + obs_vel * (dt_ + latency_) + safety_margin_;
		//printf("Dist to stop = %f\n", dist_to_stop);

		if (curr_fpl_ < dist_to_stop_) {
			printf("Decel : %f\n", curr_fpl_);
			setVel_ = std::max(0.0f, setVel_ - (max_decel_ * dt_));
		}
		else if (setVel_ < max_vel_) {
			printf("Accel : %f\n", curr_fpl_);
			setVel_ = std::min(max_vel_, setVel_ + (max_accel_ * dt_));
		}
		else {
			printf("Cruise : %f\n", curr_fpl_);
			setVel_ = max_vel_;
		}
	}

	float Navigation::CalcFreePathLen(float r, float x, float y) {
		// r is always positive
		float L = 10000;

		float rmax = sqrtf(Sq(std::abs(r) + maxPtCar_y_) + Sq(maxPtCar_x_));
		//printf("rmax : %f  ", rmax);
		float rmin = r - maxPtCar_y_;
		//printf("rmin : %f  ", rmin);
		float robs = sqrtf(Sq(x) + Sq(y - r));
		//printf("robs : %f\n", robs);

		if ((robs >= rmin) & (robs <= rmax) & (x > 0.0)) {  // there's a collision
			float ang_car;
			float ang_obs = atan2f(x, r - y);
			float rmid = sqrtf(Sq(rmin) + Sq(maxPtCar_x_));
			if (robs >= rmid) { // front collision
				ang_car = atan2f(maxPtCar_x_, rmin);
				//printf("Front Collision: ang_obs=%f , ang_car=%f , rorxrm=(%f,%f,%f) , \n", ang_obs, ang_car, robs, rmax, rmin);
				//printf("Front Collision\n");
			}
			else { // side collision
				ang_car = acosf(rmin / robs);
				//printf("Side Collision: ang_car=%f\n", ang_car);
				//printf("Side Collision\n");
			}
			//if (ang_car>ang_obs) { printf("Negative L : robs=(%f) , p = (%f , %f)\n", robs, x, y); }

			L = r * std::max(0.0f, (ang_obs - ang_car));
		}

		if ((x < 0.0) & (x >= -minPtCar_x_) & (y < 0.0)) { // tail end might collide
			float rback = sqrtf(Sq(maxPtCar_y_) + Sq(minPtCar_x_));
			float magP = sqrtf(Sq(x) + Sq(y));
			if (rback >= magP) {
				L = 0.0;
				printf("Back Collision p=(%f,%f) robs=(%f) rback=(%f)\n", x, y, robs, rback);
				sleep(3);
			}
		}

		return L;
	}

	float Navigation::CalcPathScore(float fpl, float clearance, float curvature, float r, bool isStr8) {
		if (fpl < dist_to_stop_) { return -1000.0; } // path too short: no longer an option
		if (clearance < 0.0) { return -1000.0; } // path doesn't have enough clearance: no longer an option

		float w0 = 1.0; // fpl/goal_dist_
		float w1 = 1.0 / curr_clearance_; // clearance
		float w2 = 1.0; // dist_toward_goal
		float w3 = -0.125; // delta_curvature
		float score;

		//compute dist toward goal
		Vector2f vec_path;
		if (isStr8) { vec_path = Vector2f(fpl , 0.0); } // zero curvature
		else {
			float theta = fpl / r;
			vec_path = Vector2f(r*sinf(theta) , r*(1.0-cosf(theta)));
		}
		float dist_toward_goal = immediate_goal_.norm() - (immediate_goal_-vec_path).norm();
		//float dist_toward_goal;
		//if (isStr8) { dist_toward_goal = fpl; } // zero curvature
		//else { dist_toward_goal = abs(r) * sinf(fpl * abs(curvature)); } // non-zero curvature
		
		float s1 = fpl / goal_dist_;
		float s2 = std::min(1.0f, clearance / maxClearance_);
		float s3 = dist_toward_goal / fpl;
		float s4 = Sq(Sq(curr_curvature_ - curvature));

		score = w0 * s1 + w1 * s2 + w2 * s3 + w3 * s4;
		//printf("Stats for curv %f: fpl=%f , clc=%f , dtg=%f , dcv=%f, score=%f\n", curvature, s1, s2, s3, s4, score);

		return score;
	}

	void Navigation::FindBestPath(const vector<Vector2f>& point_cloud) {
		float r, L;
		float bestFPL = goal_dist_;
		float bestCurvature = 0.0;
		float fpl = goal_dist_;
		float bestClearance = maxClearance_;
		float clearance = maxClearance_;

		printf("Curr Clearance = %f\n", curr_clearance_);
		// ----- ZERO CURVATURE -----
		for (Vector2f p : point_cloud) {
			if ((abs(p.y()) <= maxPtCar_y_) & (p.x() >= maxPtCar_x_) & (p.x() <= immediate_goal_.x())) { // observation is directly in front of me
				fpl = std::min(fpl, p.x() - maxPtCar_x_);
			}
		}
		for (Vector2f p : point_cloud) {
			if ((p.x() <= fpl + maxPtCar_x_) & (p.x() >= maxPtCar_x_)) {
				clearance = std::min(clearance, abs(p.y()) - maxPtCar_y_);
			}
		}
		float bestScore = CalcPathScore(fpl, clearance, 0.0f, 0.0f, true); // zero curvature
		bestFPL = fpl;
		bestClearance = clearance;
		DrawLine(Vector2f(0.0,0.0), Vector2f(fpl,0.0), 0xc0c0c0, local_viz_msg_);
		
		float score;
		for (float curvature = 0.05; curvature <= 1.0; curvature += 0.05) {
			r = 1 / curvature;
			clearance = maxClearance_;
			float rmax = sqrtf(Sq(r + maxPtCar_y_) + Sq(maxPtCar_x_));
			float rmin = r - maxPtCar_y_;

			// ----- POSITIVE CURVATURE -----
			if (immediate_goal_.y() > r) {
				fpl = r * std::min(M_PI_2, (M_PI - atan2(immediate_goal_.x(), immediate_goal_.y()-r)));
			}
			else {
				fpl = r * std::min((float)M_PI_2, atan2(immediate_goal_.x(), r-immediate_goal_.y())); // length of arc at min dist_goal
			}
			//printf("Orig FPL for curv %f : %f\n", curvature, fpl);
			for (Vector2f p : point_cloud) {
				L = CalcFreePathLen(r, p.x(), p.y());
				if (L < fpl) { fpl = L; }
			}
			for (Vector2f p : point_cloud) {
				clearance = std::min(clearance, CalcClearance(r, p[0], p[1], fpl, rmax, rmin));
			}
			DrawArc(Vector2f(0.0,r), r, -M_PI/2.0, fpl*curvature-(M_PI/2.0), 0xc0c0c0, local_viz_msg_);
			score = CalcPathScore(fpl, clearance, curvature, r, false);
			if (score > bestScore) {
				bestScore = score;
				bestCurvature = curvature;
				bestFPL = fpl;
				bestClearance = clearance;
			}

			// ----- NEGATIVE CURVATURE -----
			if (-immediate_goal_.y() > r) {
				fpl = r * std::min(M_PI_2, (M_PI - atan2(immediate_goal_.x(), -immediate_goal_.y()-r)));
			}
			else {
				fpl = r * std::min((float)M_PI_2, atan2(immediate_goal_.x(), r+immediate_goal_.y())); // length of arc at min dist_goal
			}
			clearance = maxClearance_;
			//printf("Orig FPL for curv %f : %f\n", -curvature, fpl);
			for (Vector2f p : point_cloud) {
				L = CalcFreePathLen(r, p[0], -p[1]);
				if (L < fpl) { fpl = L; }
			}
			for (Vector2f p : point_cloud) {
				clearance = std::min(clearance, CalcClearance(r, p[0], -p[1], fpl, rmax, rmin));
			}
			DrawArc(Vector2f(0.0,-r), r, M_PI/2.0-(fpl*curvature), M_PI/2.0, 0xc0c0c0, local_viz_msg_);
			score = CalcPathScore(fpl, clearance, -curvature, -r, false);
			if (score > bestScore) {
				bestScore = score;
				bestCurvature = -curvature;
				bestFPL = fpl;
				bestClearance = clearance;
			}
		} // loop over curvatures
		printf("Best path found to be (%f , %f)\n", bestFPL, bestCurvature);
		//}
		curr_fpl_ = bestFPL;
		curr_curvature_ = bestCurvature;
		curr_clearance_ = bestClearance;
	}

	float Navigation::CalcClearance(float r, float x, float y, float fpl, float rmax, float rmin) {
		// r is always positive
		float clearance = 100.0; // arbitrary large number
		float ang_max = fpl / r + atan2f(maxPtCar_x_, rmin); // ang from base_link to tip of car at fpl
		float robs = sqrtf(Sq(x) + Sq(y - r)); // can optimize by storing this calculation elsewhere
		float ang_obs = atan2f(x, r - y);

		if ((ang_obs > 0.0) & (ang_obs <= ang_max)) { // points in consideration
			if (robs > r) { clearance = robs - rmax; }
			else { clearance = rmin - robs; }
		}

		return clearance;
	}

	void Navigation::DrawCarAndPath() {
		DrawLine(Vector2f(-minPtCar_x_, maxPtCar_y_), Vector2f(maxPtCar_x_, maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawLine(Vector2f(maxPtCar_x_, maxPtCar_y_), Vector2f(maxPtCar_x_, -maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawLine(Vector2f(-minPtCar_x_, -maxPtCar_y_), Vector2f(maxPtCar_x_, -maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawLine(Vector2f(-minPtCar_x_, maxPtCar_y_), Vector2f(-minPtCar_x_, -maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawPathOption(curr_curvature_, curr_fpl_, curr_clearance_, local_viz_msg_);
	}

	void Navigation::DrawGraphandCarrot() {
		/*
		std::vector<int> neighbours;
		for (size_t k = 0; k < edges.size(); k++) {
			neighbours = edges[k];
			for (size_t j = 0; j < neighbours.size(); j++) {
				DrawLine(Vector2f(vertices[k].x(), vertices[k].y()), Vector2f(vertices[neighbours[j]].x(), vertices[neighbours[j]].y()), 0x620200, global_viz_msg_);
			}
		}
		*/
		for (size_t k = 0; k < path_.size() - 1; k++)
		{
			DrawLine(Vector2f(vertices[path_[k]].x(), vertices[path_[k]].y()), Vector2f(vertices[path_[k + 1]].x(), vertices[path_[k + 1]].y()), 0x620200, global_viz_msg_);
		}
		//DrawCross(immediate_goal_, 0.2, 0xFF0000, local_viz_msg_);
		DrawCross(nav_goal_loc_, 0.2, 0xFF0000, global_viz_msg_);
		//DrawArc(Vector2f(0, 0), goal_dist_, -M_PI, M_PI, 0x800080, local_viz_msg_);
	}
	
	void Navigation::IntersectionCircleLine(int start_ndx, int end_ndx) {
		//(x - line_start.x()) ^ 2 = (radius * radius - line_start.y())(line_end.x() - line_start.x()) / (line_end.y() - line_start.y());
		//float phi = atan2f(line_end.y() - odom_loc_.y() , line_end.x() - odom_loc_.x());
		//immediate_goal_.x() = line_start.x() + radius * cosf(phi);
		//immediate_goal_.y() = line_start.y() + radius * sinf(phi);
		Vector2f goal;
		float M = (vertices[path_[end_ndx]].y() - vertices[path_[start_ndx]].y()) / (vertices[path_[end_ndx]].x() - vertices[path_[start_ndx]].x());
		float c = vertices[path_[start_ndx]].y() - M * vertices[path_[start_ndx]].x();
		float a = -M;
		float b = 1.0;
		float c_dash = c - a * odom_loc_.x() - b * odom_loc_.y();
		goal.x() = (a * c_dash + b * sqrt(Sq(goal_dist_) * (Sq(a) + Sq(b)) - Sq(c_dash))) / (Sq(a) + Sq(b));
		goal.y() = (b * c_dash - a * sqrt(Sq(goal_dist_) * (Sq(a) + Sq(b)) - Sq(c_dash))) / (Sq(a) + Sq(b));
		immediate_goal_ = Rotation2Df(-odom_angle_) * goal;
		
		printf("Immediate goal=(%f,%f) , Goal=(%f,%f)\n", immediate_goal_.x(), immediate_goal_.y(), goal.x(), goal.y());
		printf("M=%f , c=%f , b=%f , c_dash=%f\n", M, c, b, c_dash);
		printf("start_ndx=%i , end_ndx=%i\n", start_ndx, end_ndx);
		printf("path[start]=%i , path[end]=%i\n", path_[start_ndx], path_[end_ndx]);
		printf("vertices[start]=(%f,%f) , vertices[end]=(%f,%f)\n", vertices[path_[start_ndx]].x(), vertices[path_[start_ndx]].y(), vertices[path_[end_ndx]].x(), vertices[path_[end_ndx]].y());
	}
	
	void Navigation::CarrotPositioning() {
		need_to_replan_ = true;
		//int pathSize_m1 = path_.size() - 1;
		float dist_to_path = goal_dist_;
		float min_dist_to_path = 100000.0;
		int closest_path_ndx = curr_path_ndx_;
		for (size_t k = curr_path_ndx_; k < path_.size(); k++) {
			dist_to_path = (vertices[path_[k]] - odom_loc_).norm();
			if (dist_to_path < min_dist_to_path) {
				min_dist_to_path = dist_to_path;
				closest_path_ndx = k;
				need_to_replan_ = false;
			}
			/*
			 * if (dist_to_path < goal_dist_) {
				need_to_replan_ = false;
				int j = k;
				while ((j<pathSize_m1) && (vertices[path_[j]] - odom_loc_).norm() < goal_dist_) {
					j++;
				}
				//line2f my_line(vertices[path_[k]].x(), vertices[path_[k]].y(), vertices[path_[j]].x(), vertices[path_[j]].y());
				IntersectionCircleLine(k, j);
				//DrawLine(vertices[path_[k]], vertices[path_[j]], 0xFF0000, global_viz_msg_);
				break;
			}
			*/
			if (dist_to_path > goal_dist_) {
				immediate_goal_ = Rotation2Df(-odom_angle_) * (vertices[path_[k]] - odom_loc_);
				break;
			}
		}
		curr_path_ndx_ = closest_path_ndx;
		
		// if immediate goal is behind base_link.x -> replan
		if (immediate_goal_.x()<0.0) { need_to_replan_ = true; }
		printf("Immediate goal=(%f,%f)\n", immediate_goal_.x(), immediate_goal_.y());
	}
	
	void Navigation::Run() {
		if (!is_odom_init_) {
			printf("odom not initialized ... skipping\n");
			return;
		}
		if (!nav_target_set_) {
			printf("nav target not set ... skipping\n");
			return;
		}
		if (!path_is_ready_) {
			printf("path isn't ready ... skipping\n");
			return;
		}
		//double curr_time = GetMonotonicTime(); // arbitrary time reference

		// Goal Reached
		if ((nav_goal_loc_ - odom_loc_).norm() < goal_dist_) {
			printf("Nav target reached!\n");
			nav_complete_ = true; // FindBestPath() no longer called
			immediate_goal_ = Rotation2Df(-odom_angle_) * (nav_goal_loc_ - odom_loc_);
			if (immediate_goal_.norm() > curr_fpl_) { curr_fpl_ = 0.0; } // stop if we start to get further away from the target
			else {
				curr_fpl_ = immediate_goal_.norm();
				//curr_curvature_ = 0.0;
			}
	    }
	    else {
			// Get the Next Carrot, Return need_to_replan_=true if no carrot available
			CarrotPositioning();
			
			// Need to Re-Calculate the Path
			if (need_to_replan_) {
				printf("Recalculating path!\n");
				path_is_ready_ = false;
				drive_msg_.velocity = 0.0; // m/s
				drive_msg_.curvature = 0.0; // 1/radius
				drive_pub_.publish(drive_msg_);
				path_parent_.clear();
				AStarSearch();
				ReconstructPath();
				CarrotPositioning();
				//Sleep(2.0);
				//return;
			}
		}
		
		DrawCarAndPath();
		DrawGraphandCarrot();
		TOC();
	    
		//printf("Odom: (%f,%f) \n", odom_loc_.x(), odom_loc_.y());
		//printf("Goal: (%f,%f) \n", nav_goal_loc_.x(), nav_goal_loc_.y());

		printf("Setting vel(%f) : Setting curv(%f)\n", setVel_, curr_curvature_);
		drive_msg_.velocity = setVel_; // m/s
		drive_msg_.curvature = curr_curvature_; // 1/radius
		drive_pub_.publish(drive_msg_);
		viz_pub_.publish(local_viz_msg_);
		ClearVisualizationMsg(local_viz_msg_);
		viz_pub_.publish(global_viz_msg_);
		ClearVisualizationMsg(global_viz_msg_);
	}

}   // namespace navigation
