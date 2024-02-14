/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <cstdio>
#include <chrono>
#include <queue>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y) * XSIZE + (X))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// Approach: a queue is used to fill the map with the cost and number of steps associated with
// traversing from any point on the map. A cmap structure is defined to represent this cost map.
// Each node in the cost map has an associated best x and y to which the robot should move to
// to move towards the nearest, most cost-efficient point in the robot trajectory.

// Future improvements: include hash set of nodes of various useful costs and lengths for each cost map node
// I.e. if I can reach a point in the trajectory in 18 steps with cost 1200, or 12 steps with cost 1800
// I want to choose between the two, or other options, depending on how much time I have left

// Struct to hold map positions that need to be expanded in the queue
struct queue_node
{
    int x;
    int y;
    float cost;
    float heuristic;
    float priority;
};

struct CompareNode
{
    bool operator()(const queue_node *lhs, const queue_node *rhs) const
    {
        return lhs->priority > rhs->priority;
    }
};

float euclidean_distance(int x1, int y1, int x2, int y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

using MinHeap = std::priority_queue<queue_node *, std::vector<queue_node *>, CompareNode>;

// Struct to represent each position on the cost map
struct map_node
{
    int cost;
    int steps;
    bool expanded;
    int best_x;
    int best_y;
    int idx_of_path_node;
};

// Struct to represent the cost map
struct cmap
{
    map_node **map;
};

queue_node *queue_node_new(int x, int y, float cost, int goalX, int goalY)
{
    queue_node *n = new queue_node;
    n->x = x;
    n->y = y;
    n->cost = cost;
    n->heuristic = euclidean_distance(x, y, goalX, goalY);
    n->priority = n->cost + n->heuristic;
    return n;
}

// Returns a new cost map
cmap *cmap_new(int x_size, int y_size)
{
    cmap *c = new cmap;
    c->map = new map_node *[x_size];
    for (int i = 0; i < x_size; i++)
    {
        c->map[i] = new map_node[x_size];
        for (int j = 0; j < y_size; j++)
        {
            c->map[i][j].cost = -1;
            c->map[i][j].steps = -1;
            c->map[i][j].expanded = false;
            c->map[i][j].best_x = -1;
            c->map[i][j].best_y = -1;
            c->map[i][j].idx_of_path_node = -1;
        }
    }
    return c;
}

// Frees the memory associated with the cost map
void free_cmap(cmap *c, int x_size)
{
    for (int j = 0; j < x_size; j++)
    {
        delete c->map[j];
    }
    delete c->map;
    delete c;
}

// Checks whether x, y is a valid position in the map and free of obstacles
bool valid_pos(int x, int y, int x_size, int y_size, int *map, int collision_thresh)
{
    return (x > 0 && y > 0 && x <= x_size && y <= y_size && map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh);
}

// Returns the cost map given parameters
cmap *fill_cmap(int *map, int *target_traj, int curr_time, int target_steps, int collision_thresh, int x_size, int y_size, int robotPoseX, int robotPoseY)
{

    MinHeap q;

    auto start = std::chrono::high_resolution_clock::now(); // Start timing

    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    cmap *c = cmap_new(x_size, y_size);

    for (int i = curr_time; i < target_steps; i++)
    {

        int x_targ_step = target_traj[i];
        int y_targ_step = target_traj[target_steps + i];
        c->map[x_targ_step - 1][y_targ_step - 1].best_x = x_targ_step;
        c->map[x_targ_step - 1][y_targ_step - 1].best_y = y_targ_step;
        c->map[x_targ_step - 1][y_targ_step - 1].steps = 0;
        c->map[x_targ_step - 1][y_targ_step - 1].cost = 0;
        c->map[x_targ_step - 1][y_targ_step - 1].expanded = true;
        c->map[x_targ_step - 1][y_targ_step - 1].idx_of_path_node = i;
        q.push(queue_node_new(x_targ_step, y_targ_step, c->map[x_targ_step - 1][y_targ_step - 1].cost, robotPoseX, robotPoseY));
    }

    while (!q.empty())
    {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed > 0.95)
        {
            return c;
        }
        queue_node *n = q.top();
        q.pop();
        int curr_x = n->x;
        int curr_y = n->y;
        delete n;
        int idx_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;

        // If the node is reachable within our time constraint
        if (c->map[curr_x - 1][curr_y - 1].steps <= abs(idx_node - curr_time))
        {
            // Explore each of the 8 points possibly connected to it
            for (int i = 0; i < NUMOFDIRS; i++)
            {
                int new_x = curr_x + dX[i];
                int new_y = curr_y + dY[i];

                // If this new point is a valid position
                if (valid_pos(new_x, new_y, x_size, y_size, map, collision_thresh))
                {
                    // If this node hasn't been visited before, initialize it
                    if (!c->map[new_x - 1][new_y - 1].expanded)
                    {
                        c->map[new_x - 1][new_y - 1].expanded = true;
                        c->map[new_x - 1][new_y - 1].best_x = curr_x;
                        c->map[new_x - 1][new_y - 1].best_y = curr_y;
                        c->map[new_x - 1][new_y - 1].cost = c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                        c->map[new_x - 1][new_y - 1].steps = c->map[curr_x - 1][curr_y - 1].steps + 1;
                        c->map[new_x - 1][new_y - 1].idx_of_path_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;
                        q.push(queue_node_new(new_x, new_y, c->map[new_x - 1][new_y - 1].cost, robotPoseX, robotPoseY));
                    }
                    else
                    {
                        bool cost_condition = c->map[new_x - 1][new_y - 1].cost > c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                        if (cost_condition)
                        {
                            c->map[new_x - 1][new_y - 1].cost = c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                            c->map[new_x - 1][new_y - 1].best_x = curr_x;
                            c->map[new_x - 1][new_y - 1].best_y = curr_y;
                            c->map[new_x - 1][new_y - 1].idx_of_path_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;
                            q.push(queue_node_new(new_x, new_y, c->map[new_x - 1][new_y - 1].cost, robotPoseX, robotPoseY));
                        }
                    }
                }
            }
        }
    }
    return c;
}

void planner(
    int *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int *target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int *action_ptr)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    // Check whether we are already at a valid point in the target's trajectory
    for (int i = curr_time; i < target_steps; i++)
    {
        if (target_traj[i] == robotposeX && target_traj[i + target_steps] == robotposeY)
        {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
    }

    // Fill the cost map
    cmap *c = fill_cmap(map, target_traj, curr_time, target_steps, collision_thresh, x_size, y_size, robotposeX, robotposeY);

    // If our current position has been expanded (i.e. we can feasibly reach a point on the target trajectory from it)
    // Move to the best x, y calculated during the cmap fill
    if (c->map[robotposeX - 1][robotposeY - 1].expanded)
    {
        printf("expanded\n");
        action_ptr[0] = c->map[robotposeX - 1][robotposeY - 1].best_x;
        action_ptr[1] = c->map[robotposeX - 1][robotposeY - 1].best_y;
    }
    // We can't catch the target, and just stay still
    else
    {
        printf("not expanded\n");
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    free_cmap(c, x_size);
    return;
}
