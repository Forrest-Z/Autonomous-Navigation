In this assignment, we implemented a navigation planner, and integrated it with obstacle avoidance and particle filter localization algorithms to allow the robot to autonomously drive around an indoor environment. We used starter code given in the course Autonomous Robots.

## Algorithm Description
1. Graph creation: Once the map is loaded all possible vertices are created at a given resolution. The graph is created by imposing an 8-connected grid on all the vertices. Graph edges that intersect map lines are removed. 
2. To avoid paths too close to mapped objects, we place “virtual” lines a safe_distance from objects in four directions. This also resolves the problem of map lines which nearly, but do not completely, intersect. 
3. Once the pruned graph is created, we run the A* Search Algorithm as our global path planner. For the heuristic we use euclidean distance, because it is admissible and easily computed. Additionally, we inflated the heuristic by 10% for computational efficiency, at the cost of potentially suboptimal paths. However, since this inflation is so small, deviations from optimality will be negligible. Also, we changed the given Priority Queue from max queue to min queue by adjusting the comparator. 
4. To begin, the function SearchNearestPoint finds the nearest available vertices for the starting point and the goal point. This function returns the vertices nearest the starting point and target point. 
5. The path is constructed by tracking the came_from list in reverse order. The first trackable vertex is the goal. The interaction will stop when it reaches the starting point. The path is reversed to ensure the correct direction. 
6. Once we have a global path, we find the position of the carrot using a simple carrot-follower algorithm. The carrot is chosen as the next vertex on the path that is at-least a minimum distance from the current pose. 
7. The location of the carrot is transformed in to the local base_link frame as follows: (− Xlocal = R θmap)(carrot − Xmap) where : R(θ) is a 2D rotation matrix carrot is a 2D column vector containing the (x, y) position of the carrot in the map frame X is a 2D column vector containing an (x, y) position 
8. The current path is valid if the carrot is within a given distance from the vehicle. Otherwise, a new path is planned. Also, if the carrot is behind the vehicle the path is invalidated and a new path is planned. This happens, for example, in the case that an obstacle is present which does not allow the vehicle to follow the path. This case can be seen in two of our demonstration videos.

### Clone and Build
1. `mkdir projects`
1. `cd projects`
1. `git clone <your repository url>` (found in the upper right)
1. `cd <cloned_repo>`
1. `make -j`
