/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <cstdio>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct queue_node
{
    int x;
    int y;
    queue_node *next;
};

struct map_node
{
    int cost;
    int steps;
    bool expanded;
    int best_x;
    int best_y;
    int idx_of_path_node;
};

struct cmap
{
    map_node **map;
    int initialized_size;
};

struct queue
{
    queue_node *first;
    queue_node *last;
    int size;
};

bool queue_empty(queue *q)
{
    return (q->size == 0);
}

void push(queue *q, queue_node *n)
{
    if (queue_empty(q))
    {
        q->first = n;
        q->last = n;
    }
    else
    {
        n->next = NULL;
        q->last->next = n;
        q->last = n;
    }
    q->size++;
}

queue_node *pop(queue *q)
{
    if (!queue_empty(q))
    {
        queue_node *n = q->first;
        q->first = q->first->next;
        q->size--;
        return n;
    }
    else
    {
        return NULL;
    }
}

queue *queue_new()
{
    queue *q = new queue;
    q->first = NULL;
    q->last = NULL;
    q->size = 0;
    return q;
}

queue_node *queue_node_new(int x, int y)
{
    queue_node *n = new queue_node;
    n->x = x;
    n->y = y;
    n->next = NULL;
    return n;
}

cmap *cmap_new(int x_size, int y_size)
{
    cmap *c = new cmap;
    c->map = new map_node *[x_size];
    c->initialized_size = 0;
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

void free_cmap(cmap *c, int x_size)
{
    for (int j = 0; j < x_size; j++)
    {
        delete c->map[j];
    }
    delete c->map;
    delete c;
}

void clear_queue(queue *q)
{
    while (!queue_empty(q))
    {
        delete pop(q);
    }
}

queue *copy_queue(queue *q)
{
    queue *nq = queue_new();
    while (nq->size < q->size)
    {
        queue_node *n = pop(q);
        push(q, n);
        push(nq, n);
    }
    return nq;
}

bool valid_pos(int x, int y, int x_size, int y_size, int *map, int collision_thresh)
{
    return (x > 0 && y > 0 && x <= x_size && y <= y_size && map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh);
}

cmap *fill_cmap(int *map, int *target_traj, int curr_time, int target_steps, int collision_thresh, int x_size, int y_size)
{
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    cmap *c = cmap_new(x_size, y_size);
    queue *q = queue_new();

    int max_steps = target_steps - curr_time;

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
        push(q, queue_node_new(x_targ_step, y_targ_step));
    }

    while (!queue_empty(q))
    {
        queue_node *n = pop(q);
        int curr_x = n->x;
        int curr_y = n->y;
        delete n;
        int idx_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;

        if (c->map[curr_x - 1][curr_y - 1].steps <= abs(idx_node - curr_time))
        {
            for (int i = 0; i < NUMOFDIRS; i++)
            {
                int new_x = curr_x + dX[i];
                int new_y = curr_y + dY[i];
                if (valid_pos(new_x, new_y, x_size, y_size, map, collision_thresh))
                {
                    if (!c->map[new_x - 1][new_y - 1].expanded)
                    {
                        c->map[new_x - 1][new_y - 1].expanded = true;
                        c->map[new_x - 1][new_y - 1].best_x = curr_x;
                        c->map[new_x - 1][new_y - 1].best_y = curr_y;
                        c->map[new_x - 1][new_y - 1].cost = c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                        c->map[new_x - 1][new_y - 1].steps = c->map[curr_x - 1][curr_y - 1].steps + 1;
                        c->map[new_x - 1][new_y - 1].idx_of_path_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;
                        push(q, queue_node_new(new_x, new_y));
                    }
                    else
                    {
                        if (c->map[new_x - 1][new_y - 1].steps > c->map[curr_x - 1][curr_y - 1].steps + 1)
                        {
                            c->map[new_x - 1][new_y - 1].steps = c->map[curr_x - 1][curr_y - 1].steps + 1;
                            c->map[new_x - 1][new_y - 1].cost = c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                            c->map[new_x - 1][new_y - 1].best_x = curr_x;
                            c->map[new_x - 1][new_y - 1].best_y = curr_y;
                            c->map[new_x - 1][new_y - 1].idx_of_path_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;
                            push(q, queue_node_new(new_x, new_y));
                        }
                        else if (c->map[new_x - 1][new_y - 1].steps == c->map[curr_x - 1][curr_y - 1].steps + 1)
                        {
                            if (c->map[new_x - 1][new_y - 1].cost > c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)])
                            {
                                c->map[new_x - 1][new_y - 1].cost = c->map[curr_x - 1][curr_y - 1].cost + map[GETMAPINDEX(new_x, new_y, x_size, y_size)];
                                c->map[new_x - 1][new_y - 1].best_x = curr_x;
                                c->map[new_x - 1][new_y - 1].best_y = curr_y;
                                c->map[new_x - 1][new_y - 1].idx_of_path_node = c->map[curr_x - 1][curr_y - 1].idx_of_path_node;
                                push(q, queue_node_new(new_x, new_y));
                            }
                        }
                    }
                }
            }
        }
    }
    delete q;

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

    for (int i = curr_time; i < target_steps; i++)
    {
        if (target_traj[i] == robotposeX && target_traj[i + target_steps] == robotposeY)
        {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
    }
    cmap *c = fill_cmap(map, target_traj, curr_time, target_steps, collision_thresh, x_size, y_size);

    if (c->map[robotposeX - 1][robotposeY - 1].expanded)
    {
        action_ptr[0] = c->map[robotposeX - 1][robotposeY - 1].best_x;
        action_ptr[1] = c->map[robotposeX - 1][robotposeY - 1].best_y;
    }
    else
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    free_cmap(c, x_size);
    return;
}