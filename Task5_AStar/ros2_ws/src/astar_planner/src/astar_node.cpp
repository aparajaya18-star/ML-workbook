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
		return abs(x - endNode->x) + abs(y - endNode->y); //Manhattan distance 
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
		return a->fCost > b->fCost; // Min-heap based on fCost
	}
};

vector<AStarNode*> aStar(
    vector<vector<int>>& grid,
    AStarNode* startNode,
    AStarNode* endNode
)
{
	vector<AStarNode*> openList;
	set<pair<int, int>> closedList;

	openList.push_back(startNode);

	while (!openList.empty())
	{
		sort(openList.begin(), openList.end(), CompareNode());
		AStarNode* currentNode = openList.front();
		openList.erase(openList.begin());

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

		vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
		for (auto dir : directions)
		{
			int newX = currentNode->x + dir.first;
			int newY = currentNode->y + dir.second;
			AStarNode* neighbor = new AStarNode(newX, newY);

			if (!neighbor->isWalkable(grid) || closedList.count({newX, newY}) > 0)
				continue;

			int tentativeGCost = currentNode->gCost + 1;

			auto it = find_if(openList.begin(), openList.end(), [&](AStarNode* node) {
				return node->x == newX && node->y == newY;
			});

			if (it == openList.end() || tentativeGCost < (*it)->gCost)
			{
				neighbor->gCost = tentativeGCost;
				neighbor->hCost = neighbor->calculateHeuristic(endNode);
				neighbor->fCost = neighbor->gCost + neighbor->hCost;
				neighbor->parent = currentNode;

				if (it == openList.end())
					openList.push_back(neighbor);
			}
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

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
		RCLCPP_INFO(this->get_logger(), "Map received!");
        int width = msg->info.width;
        int height = msg->info.height;

        vector<vector<int>> grid(height, vector<int>(width));
		RCLCPP_INFO(this->get_logger(), "Start value: %d", grid[0][0]);
		RCLCPP_INFO(this->get_logger(), "Goal value: %d", grid[height-1][width-1]);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = y * width + x;
                grid[y][x] = (msg->data[i] == 0) ? 0 : 1;
            }
        }

        AStarNode* start = new AStarNode(0, 0);
        AStarNode* goal = new AStarNode(width - 1, height - 1);

        auto path = aStar(grid, start, goal);
		RCLCPP_INFO(this->get_logger(), "Path size: %ld", path.size());

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
		path_msg.header.stamp = this->now();

        for (auto node : path)
        {
            geometry_msgs::msg::PoseStamped pose;

			pose.header.frame_id = "map";
			pose.header.stamp = this->now();

			pose.pose.position.x = node->x;
			pose.pose.position.y = node->y;
			pose.pose.position.z = 0.0;

			pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

		RCLCPP_INFO(this->get_logger(), "Publishing path...");
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<AStarPlanner>());
	rclcpp::shutdown();
	return 0;
}