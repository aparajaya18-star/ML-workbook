#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <algorithm>

using namespace std;

// 0 = free space, 1 = obstacle
vector<vector<int>> grid = {
	{0, 0, 0, 0, 0},
	{0, 1, 1, 1, 0},
	{0, 0, 0, 1, 0},
	{0, 1, 0, 0, 0},
	{0, 0, 0, 1, 0}
};

struct Node {
	int x, y;
	int gCost, hCost, fCost;
	Node* parent;

	Node(int x, int y)
	{
		this->x = x;
		this->y = y;
		gCost = hCost = fCost = 0;
		parent = nullptr;
	}
	float calculateHeuristic(Node* endNode)
	{
		return abs(x - endNode->x) + abs(y - endNode->y); //Manhattan distance 
	}
	bool isWalkable()
	{
		return (x>= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0);
		// Check if the node is within grid bounds and not an obstacle
	}
};

struct CompareNode {
	bool operator()(Node* a, Node* b)
	{
		return a->fCost > b->fCost; // Min-heap based on fCost
	}
};

vector<Node*> aStar(Node* startNode, Node* endNode)
{
	vector<Node*> openList;
	set<pair<int, int>> closedList;

	openList.push_back(startNode);

	while (!openList.empty())
	{
		sort(openList.begin(), openList.end(), CompareNode());
		Node* currentNode = openList.front();
		openList.erase(openList.begin());

		// Check if we have reached the end node
		if (currentNode->x == endNode->x && currentNode->y == endNode->y)
		{
			vector<Node*> path;
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
			Node* neighbor = new Node(newX, newY);

			if (!neighbor->isWalkable() || closedList.count({newX, newY}) > 0)
				continue;

			int tentativeGCost = currentNode->gCost + 1;

			auto it = find_if(openList.begin(), openList.end(), [&](Node* node) {
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

	return vector<Node*>(); // Return empty path if no path is found
}

int main()
{
	Node* startNode = new Node(0, 0);
	Node* endNode = new Node(4, 4);

	vector<Node*> path = aStar(startNode, endNode);

	if (!path.empty())
	{
		cout << "Path found:" << endl;
		for (Node* node : path)
		{
			cout << "(" << node->x << ", " << node->y << ") ";
		}
		cout << endl;
	}
	else
	{
		cout << "No path found." << endl;
	}

	return 0;
}
