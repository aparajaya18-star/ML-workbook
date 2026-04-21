import heapq
import matplotlib.pyplot as plt
import numpy as np
import cv2

def resize_grid(grid, scale=0.25):
    h, w = grid.shape
    new_size = (int(w*scale), int(h*scale))
    small = cv2.resize(grid, new_size, interpolation=cv2.INTER_NEAREST)
    return small

mask = cv2.imread("segmented_mask.png", cv2.IMREAD_GRAYSCALE)

grid = (mask == 255).astype(np.uint8) 
grid = 1 - grid
grid = resize_grid(grid, 0.2)

plt.figure(figsize=(10,5))
plt.subplot(1,2,1)
plt.imshow(mask)
plt.title("Original")

plt.subplot(1,2,2)
plt.imshow(grid, cmap='gray')
plt.title("Grid (0=road, 1=obstacle)")
plt.show()

class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f  # for heapq

def heuristic(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5  # Euclidean

def is_walkable(x, y):
    return (0 <= x < grid.shape[0] and 
            0 <= y < grid.shape[1] and 
            grid[x][y] == 0)

def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def astar(start, end):
    open_heap = []
    heapq.heappush(open_heap, Node(start[0], start[1]))

    closed_set = set()
    visited = {}  # (x,y) -> g cost

    while open_heap:
        current = heapq.heappop(open_heap)

        if (current.x, current.y) == end:
            return reconstruct_path(current)

        closed_set.add((current.x, current.y))

        directions = [
            (0,1), (1,0), (0,-1), (-1,0),
            (1,1), (1,-1), (-1,1), (-1,-1)
        ]

        for dx, dy in directions:
            nx, ny = current.x + dx, current.y + dy

            if not is_walkable(nx, ny) or (nx, ny) in closed_set:
                continue

            if dx != 0 and dy != 0:
                new_g = current.g + 1.414  # diagonal
            else:
                new_g = current.g + 1

            if (nx, ny) not in visited or new_g < visited[(nx, ny)]:
                visited[(nx, ny)] = new_g
                h = heuristic(nx, ny, end[0], end[1])
                neighbor = Node(nx, ny, new_g, h, current)
                heapq.heappush(open_heap, neighbor)

    return []  # no path

start = (0, 0)
end = (grid.shape[0]-1, grid.shape[1]-1)

if not is_walkable(*start) or not is_walkable(*end):
    print("Start or End is not on a road")
else: 
    path = astar(start, end)

    if path:
        print("Path found:")
        print(path)
    else:
        print("No path found")

    vis = cv2.cvtColor(grid*255, cv2.COLOR_GRAY2BGR)

    for x, y in path:
        vis[x, y] = [255, 0, 0]  # red path

    plt.imshow(vis)
    plt.title("A* Path")
    plt.show()

    orig = cv2.imread("topdown_image.png")
    orig = cv2.resize(orig, (grid.shape[1], grid.shape[0]))

    for x, y in path:
        orig[x, y] = [0, 0, 255]

    plt.imshow(cv2.cvtColor(orig, cv2.COLOR_BGR2RGB))
    plt.title("Final Path on Map")
    plt.axis('off')
    plt.show()