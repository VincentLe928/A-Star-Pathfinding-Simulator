import heapq

grid = [
    [0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (4, 4)
rows = len(grid)
columns = len(grid[0])

# Up, Down, Left, Right
directions = [
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1)
]

# Check if the position is inside the grid
def in_bounds(r, c):
    return 0 <= r < rows and 0 <= c < columns

# Check if the position is safe to enter
def can_go(r, c):
    if not in_bounds(r, c):
        return False

    if grid[r][c] == 1:
        return False

    return True

# Manhattan distance
def heuristic(current, goal):
    row_distance = abs(goal[0] - current[0])
    column_distance = abs(goal[1] - current[1])

    return row_distance + column_distance

# A* pathfinding algorithm
def astar(start, goal):
    if not can_go(*start) or not can_go(*goal):
        return None

    open_list = []
    g_cost = {start: 0}
    came_from = {}

    f_cost = heuristic(start, goal)
    heapq.heappush(open_list, (f_cost, start))

    while open_list:
        priority, current = heapq.heappop(open_list)

        # Check if we reached the goal
        if current == goal:
            path = [current]

            # Reconstruct the path
            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path

        # Check all four directions
        for dr, dc in directions:
            new_row = current[0] + dr
            new_column = current[1] + dc

            if can_go(new_row, new_column):
                neighbor = (new_row, new_column)

                # Distance traveled from start
                new_g = g_cost[current] + 1

                # Only update if the new route is better
                if neighbor not in g_cost or new_g < g_cost[neighbor]:
                    g_cost[neighbor] = new_g
                    came_from[neighbor] = current

                    # Calculate A* costs
                    new_h = heuristic(neighbor, goal)
                    new_f = new_g + new_h

                    # Add neighbor to priority queue
                    heapq.heappush(open_list, (new_f, neighbor))

    return None

# Print the grid in the terminal
def print_grid(path):
    for r in range(rows):
        for c in range(columns):
            position = (r, c)

            if position == start:
                print("S", end = " ")
            elif position == goal:
                print("G", end = " ")
            elif grid[r][c] == 1:
                print("#", end = " ")
            elif path is not None and position in path:
                print("*", end =" ")
            else:
                print(".", end = " ")

        print()

# Run the program
print("Start:", start)
print("Goal:", goal)

path = astar(start, goal)

if path is not None:
    print("\nPath found!")
    print("Coordinates:", path)
    print("Total steps:", len(path) - 1)
else:
    print("\nNo path found!")

print("\nGrid:")
print_grid(path)
