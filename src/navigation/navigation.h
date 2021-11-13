
#include <vector>
#include <unordered_map>
#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
    class NodeHandle;
}  // namespace ros

namespace navigation {

    struct PathOption {
        float curvature;
        float clearance;
        float free_path_length;
        Eigen::Vector2f obstruction;
        Eigen::Vector2f closest_point;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    /*  struct edge {
          std::vector<int> neighbour_id;
          edge(const int a) :neighbour_id(a) {}
      };*/




    class Navigation {
    public:

        // Constructor
        explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

        // Used in callback from localization to update position.
        void UpdateLocation(const Eigen::Vector2f& loc, float angle);

        //Load the map
        void InitializeMap(const std::string& map_file);

        //Create the Grid
        //Line segments correspond to obstacles in the world, grapgh of uniform grid, remove those edges which collide with the map
        void createGrid();

        void neighbors(const Eigen::Vector2f vertix, std::vector<int>& neighbour_);

        double Heuristic(Eigen::Vector2f& a, Eigen::Vector2f& b);

        int SearchNearestPoint(Eigen::Vector2f pos);

        //void AStarSearch(Eigen::Vector2f start, Eigen::Vector2f goal, std::unordered_map<int, int>& came_from, std::unordered_map<int, double>& cost_so_far);
        void AStarSearch();
        void ReconstructPath();

        void CarrotPositioning();
        void IntersectionCircleLine(int start_ndx, int end_ndx);

        // Used in callback for odometry messages to update based on odometry.
        void UpdateOdometry(const Eigen::Vector2f& loc,
            float angle,
            const Eigen::Vector2f& vel,
            float ang_vel);

        // Updates based on an observed laser scan
        void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
            double time);

        // Main function called continously from main
        void Run();
        // Used to set the next target pose.
        void SetNavGoal(const Eigen::Vector2f& loc, float angle);
        // Time Optimal Control
        void TOC();
        // Calculate Free Path Length
        float CalcFreePathLen(float r, float x, float y);
        // Calculate Path Score
        float CalcPathScore(float fpl, float clearance, float curvature, float r, bool isStr8);
        // Find best curvature, with it's FPL, given a point p=(x,y)
        void FindBestPath(const std::vector<Eigen::Vector2f>& p);
        // Calculate Clearance
        float CalcClearance(float r, float x, float y, float fpl, float rmax, float rmin);
        // Draw car outline and selected path
        void DrawCarAndPath();
        // Draw Graph with edges
        void DrawGraphandCarrot();

    private:

        // Current robot location in Map ref frame
        Eigen::Vector2f robot_loc_;
        // Current robot orientation in Map ref frame
        float robot_angle_;
        // Current robot velocity in base_link ref frame
        Eigen::Vector2f robot_vel_;
        // Current robot angular speed in base_link ref frame
        float robot_omega_;
        // Odometry-reported robot location.
        Eigen::Vector2f odom_loc_;
        // Odometry-reported robot angle.
        float odom_angle_;

        // Whether navigation is complete.
        bool nav_complete_ = false;
        // Navigation goal location.
        Eigen::Vector2f nav_goal_loc_;
        // Navigation goal angle.
        float nav_goal_angle_;

        /* MASC STUFF */
        Eigen::Vector2f initial_odom_loc_; // Initial odometry-reported robot location.
        float initial_odom_angle_; // Initial odometry-reported robot angle.
        bool is_odom_init_ = false; // Has the odometry been set yet?
        bool nav_target_set_ = false; // Has a nav target been identified?
        float dt_ = 0.05; // Length of one iteration
        float setVel_ = 0.0; // m/s
        float max_accel_ = 4.0; // m/s/s
        float max_decel_ = 4.0; // m/s/s
        float max_vel_ = 2.0; // m/s
        float latency_ = 0.015; // sec
        float safety_margin_ = 0.05; // meters
        float maxPtCar_x_ = (0.535 + 0.324) / 2 + safety_margin_;
        float minPtCar_x_ = (0.535 - 0.324) / 2 + safety_margin_;
        float maxPtCar_y_ = 0.281 / 2 + safety_margin_;
        float maxSensorRange_ = 10.0; // m
        float goal_dist_ = 2.5; // meters
        float curr_fpl_ = 0.0; // meters
        float curr_curvature_ = 0.0; // 1/m
        float curr_clearance_ = 2.0; // meters
        float maxClearance_ = 1.5; // I don't care if it's ever more than this
        float dist_to_stop_; // meters
		bool need_to_replan_ = true; // we have deviated beyond goal_dist_ from the path
		int curr_path_ndx_ = 0;
		bool path_is_ready_ = false;
		
		float grid_resolution_ = 0.5;
		float safe_distance_ = maxPtCar_y_;
		
        //Grid variables
        std::vector<Eigen::Vector2f> vertices;
        std::vector <std::vector<int>> edges;
        std::vector<int> path_;
        vector_map::VectorMap map_;
        Eigen::Vector2f start_loc_;
        std::unordered_map<int, int> path_parent_;
        //std::unordered_map<int, double> cost_so_far_;
        int carrot_id;
        int start_id_;
        int goal_id_;
        Eigen::Vector2f immediate_goal_;

    };

}  // namespace navigation

#endif  // NAVIGATION_H
