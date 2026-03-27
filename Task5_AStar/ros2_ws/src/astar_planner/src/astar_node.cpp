#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std;

struct AStarNode {
	int x, y;
	int gCost, hCost, fCost;
	AStarNode* parent;

	AStarNode(int x, int y) : x(x), y(y), gCost(0), hCost(0), fCost(0), parent(nullptr) {}

	float calculateHeuristic(AStarNode* endNode)
	{
		return sqrt(pow(x - endNode->x, 2) + pow(y - endNode->y, 2)) * 10;
		// Euclidean distance multiplied by 10 to match movement costs
	}

	bool isWalkable(vector<vector<int>>& grid)
	{
		return (x>= 0 && x < (int)grid[0].size() && 
                y >= 0 && y < (int)grid.size() &&         
                grid[y][x] == 0);
		// Check if the node is within grid bounds and not an obstacle
	}
};

struct CompareNode {
	bool operator()(AStarNode* a, AStarNode* b)
	{
		if (a->fCost == b->fCost)
            return a->hCost > b->hCost; // prefer closer to goal
		return a->fCost > b->fCost; // Min-heap based on fCost
	}
};

vector<AStarNode*> aStar(
    vector<vector<int>>& grid,
    AStarNode* startNode,
    AStarNode* endNode
)
{
	priority_queue<AStarNode*, vector<AStarNode*>, CompareNode> openList;
	set<pair<int, int>> closedList;

	openList.push(startNode);

	while (!openList.empty())
	{
		AStarNode* currentNode = openList.top();
		openList.pop();

		if (closedList.count({currentNode->x, currentNode->y}) > 0)
    		continue; // Skip if already processed

		// Check if we have reached the end node
		if (currentNode->x == endNode->x && currentNode->y == endNode->y)
		{
			vector<AStarNode*> path;
			while (currentNode != nullptr)
			{
				path.push_back(currentNode);
				currentNode = currentNode->parent;
			}
			reverse(path.begin(), path.end());
			return path; // Return the found path
		}

		closedList.insert({currentNode->x, currentNode->y});

		vector<pair<int, int>> directions = {
			{0, 1}, {1, 0}, {0, -1}, {-1, 0}, // Cardinal movements
			{1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonal movements
		};
		for (auto dir : directions)
		{
			int newX = currentNode->x + dir.first;
			int newY = currentNode->y + dir.second;

			if (newX < 0 || newX >= (int)grid[0].size() ||
				newY < 0 || newY >= (int)grid.size())
				continue;

			bool isDiagonal = (dir.first != 0 && dir.second != 0);
			if (isDiagonal)
			{
				int x1 = currentNode->x + dir.first;
				int y1 = currentNode->y;
				int x2 = currentNode->x;
				int y2 = currentNode->y + dir.second;

				// Check if both adjacent cardinal cells are walkable for diagonal movement
				if (grid[y1][x1] != 0 || grid[y2][x2] != 0)
					continue;
			}

			AStarNode* neighbor = new AStarNode(newX, newY);

			if (!neighbor->isWalkable(grid) || closedList.count({newX, newY}) > 0)
				continue;

			int moveCost = isDiagonal ? 14 : 10;
			int tentativeGCost = currentNode->gCost + moveCost;

			
			neighbor->gCost = tentativeGCost;
			neighbor->hCost = neighbor->calculateHeuristic(endNode);
			neighbor->fCost = neighbor->gCost + neighbor->hCost;
			neighbor->parent = currentNode;

			openList.push(neighbor);
		}
	}

	return vector<AStarNode*>(); // Return empty path if no path is found
}

class AStarPlanner : public rclcpp::Node
{
public:
    AStarPlanner() : Node("astar_planner")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(10));
		qos.transient_local();  // <-- IMPORTANT
		qos.reliable();

		sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"/map", qos,
			std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1)
		);

        rclcpp::QoS path_qos(rclcpp::KeepLast(10));
		path_qos.transient_local();
		path_qos.reliable();

		path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/astar_path", path_qos);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

	void publishPath(const vector<AStarNode*>& path,
                     const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        for (auto node : path)
        {
            geometry_msgs::msg::PoseStamped pose;

            pose.header.frame_id = "map";
            pose.header.stamp = this->now();

            pose.pose.position.x = origin_x + node->x * resolution;
            pose.pose.position.y = origin_y + node->y * resolution;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing path...");
        path_pub_->publish(path_msg);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
		RCLCPP_INFO(this->get_logger(), "Map received!");
        int width = msg->info.width;
        int height = msg->info.height;

        vector<vector<int>> grid(height, vector<int>(width));
		
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = y * width + x;
                grid[y][x] = (msg->data[i] >= 0 && msg->data[i] < 50) ? 0 : 1;
            }
        }

		vector<pair<pair<int,int>, pair<int,int>>> corner_pairs = {
			{{0,0}, {width-1,height-1}},
			{{0,height-1}, {width-1,0}}
		};

		for (auto& pair : corner_pairs)
		{
			auto [sx, sy] = pair.first;
			auto [gx, gy] = pair.second;

			if (grid[sy][sx] == 0 && grid[gy][gx] == 0)
			{
				AStarNode* start = new AStarNode(sx, sy);
				AStarNode* goal  = new AStarNode(gx, gy);

				auto path = aStar(grid, start, goal);

				if (!path.empty())
				{
					RCLCPP_INFO(this->get_logger(), "Using corner-based path");
					publishPath(path, msg);
					RCLCPP_INFO(this->get_logger(), "Path size: %lu", path.size());
					return;
				}
			}
		}

		vector<pair<int,int>> boundary_points;

		// Collect all free boundary cells
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				if (x == 0 || y == 0 || x == width-1 || y == height-1)
				{
					if (grid[y][x] == 0)
						boundary_points.push_back({x,y});
				}
			}
		}

		if (boundary_points.size() < 2)
		{
			RCLCPP_ERROR(this->get_logger(), "Not enough boundary openings!");
			return;
		}
		else if (boundary_points.size() > 200)
		{
			random_shuffle(boundary_points.begin(), boundary_points.end());
			boundary_points.resize(200); // Limit to 200 points for efficiency
		}

		vector<AStarNode*> best_path;
		AStarNode* final_start = nullptr;
		AStarNode* final_goal = nullptr;

		for (size_t i = 0; i < boundary_points.size(); i++)
		{
			for (size_t j = i+1; j < boundary_points.size(); j++)
			{
				AStarNode* start = new AStarNode(boundary_points[i].first, boundary_points[i].second);
				AStarNode* goal  = new AStarNode(boundary_points[j].first, boundary_points[j].second);

				int dx = abs(start->x - goal->x);
				int dy = abs(start->y - goal->y);

				if (dx + dy < 50) continue; // skip close points
				vector<AStarNode*> current_path = aStar(grid, start, goal);

				if (!current_path.empty() && current_path.size() > best_path.size())
				{
					best_path = current_path;
					final_start = start;
					final_goal = goal;

					RCLCPP_INFO(this->get_logger(), "Valid path found between (%d,%d) and (%d,%d)", final_start->x, final_start->y, final_goal->x, final_goal->y);
				}
			}
		}

		if (best_path.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "No valid path found!");
			return;
		}

		vector<AStarNode*> path = best_path;

		RCLCPP_INFO(this->get_logger(), "Path size: %lu", path.size());
		publishPath(path, msg);
    }
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<AStarPlanner>());
	rclcpp::shutdown();
	return 0;
}