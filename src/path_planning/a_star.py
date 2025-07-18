import cv2
import numpy as np
import heapq

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(occupancy, start, goal):
    neighbors = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1), (1,-1), (-1,1), (1,1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0]+i, current[1]+j)
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if (0 <= neighbor[0] < occupancy.shape[1] and
                0 <= neighbor[1] < occupancy.shape[0]):
                if not occupancy[neighbor[1], neighbor[0]]:  # obstacle
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

# 1. Load the occupancy grid image (black=obstacle, white=free)
img = cv2.imread('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/4_channels/occupancy_grid_4channels.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    raise ValueError("Image not found.")

# 2. Convert to binary occupancy grid: True=free, False=obstacle
occupancy = img > 127  # You may use 127 or similar threshold

# 3. Define start and goal
start = (50, 50)        # (x, y) coordinates - change as needed!
goal = (1500, 800)      # (x, y) coordinates - change as needed!

# 4. Run A*
path = astar(occupancy, start, goal)
if path is None:
    print("No path found!")
else:
    print(f"Found path with {len(path)} steps.")

    # 5. Visualize path
    img_path = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for p in path:
        cv2.circle(img_path, p, 2, (0,0,255), -1)  # draw path in red
    cv2.circle(img_path, start, 8, (0,255,0), -1)  # start green
    cv2.circle(img_path, goal, 8, (255,0,0), -1)   # goal blue
    cv2.imwrite('/Users/alessioborgi/Documents/GitHub/Tohoku_TESP/img/path_astar.png', img_path)
    print("Saved path image to path_astar.png")

# Optionally: Show path
# cv2.imshow("Path", img_path)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
