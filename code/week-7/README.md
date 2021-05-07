## HW_week7 hybrid a*

# Hybrid A* Algorithm
---

    def expand(self, current, goal):
            g = current['g']
            x, y, theta = current['x'], current['y'], current['t']

            # The g value of a newly expanded cell increases by 1 from the
            # previously expanded cell.
            g_2 = g + 1
            next_states = []

            # Consider a discrete selection of steering angles.
            for delta_t in range(self.omega_min, self.omega_max + self.omega_step, self.omega_step):
                # TODO: implement the trajectory generation based on
                # a simple bicycle model.
                # Let theta2 be the vehicle's heading (in radian)
                # between 0 and 2 * PI.
                # Check validity and then add to the next_states list.
                delta_t *= (np.pi / 180)
                omega = self.speed / self.length * np.tan(delta_t)
                theta2 = self.normalize(theta + omega)
                x_2 = x + self.speed * np.cos(theta2)
                y_2 = y + self.speed * np.sin(theta2)

                if 0 <= self.idx(x_2) < self.dim[1] and 0 <= self.idx(y_2) < self.dim[2]:
                    f_2 = g_2 + self.heuristic(x_2, y_2, goal)
                    expanded_state = self.State(x_2, y_2, theta2, g_2, f_2)
                    next_states.append(expanded_state)
            return next_states

    def search(self, grid, start, goal):
            # Initial heading of the vehicle is given in the
            # last component of the tuple start.
            theta = start[-1]
            # Determine the cell to contain the initial state, as well as
            # the state itself.
            stack = self.theta_to_stack_num(theta)
            g = 0
            s = {
                'f': self.heuristic(start[0], start[1], goal),
                'g': g,
                'x': start[0],
                'y': start[1],
                't': theta,
            }
            self.final = s
            # Close the initial cell and record the starting state for
            # the sake of path reconstruction.
            self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
            self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
            total_closed = 1
            opened = [s]

            # Examine the open list, according to the order dictated by
            # the heuristic function.
            cycle = 0
            while len(opened) > 0:
                # TODO: implement prioritized breadth-first search
                # for the hybrid A* algorithm.
                cycle += 1
                print(cycle)

                opened.sort(key=lambda s : s['f'], reverse=True)
                curr = opened.pop()
                x, y = curr['x'], curr['y']
                if (self.idx(x), self.idx(y)) == goal:
                    self.final = curr
                    found = True
                    break

                # Compute reachable new states and process each of them.
                next_states = self.expand(curr, goal)
                for n in next_states:
                    idx_x, idx_y = self.idx(n['x']), self.idx(n['y'])
                    stack = self.theta_to_stack_num(n['t'])
                    if grid[idx_x, idx_y] == 0:
                        dist_x, dist_y = abs(self.idx(x) - idx_x), abs(self.idx(y) - idx_y)
                        min_x, min_y = np.minimum(self.idx(x), idx_x), np.minimum(self.idx(y), idx_y)
                        possible = True
                        for d_x in range(dist_x + 1):
                            for d_y in range(dist_y + 1):
                                if grid[min_x + d_x, min_y + d_y] != 0:
                                    possible = False
                        if possible and self.closed[stack][idx_x][idx_y] == 0:
                            self.closed[stack][idx_x][idx_y] = 1
                            total_closed += 1
                            self.came_from[stack][idx_x][idx_y] = curr
                            opened.append(n)

            else:
                # We weren't able to find a valid path; this does not necessarily
                # mean there is no feasible trajectory to reach the goal.
                # In other words, the hybrid A* algorithm is not complete.
                found = False
            print(f"speed: {self.speed} NUM_THETA_CELLS: {self.NUM_THETA_CELLS}")
            return found, total_closed

     def theta_to_stack_num(self, theta):
            # TODO: implement a function that calculate the stack number
            # given theta represented in radian. Note that the calculation
            # should partition 360 degrees (2 * PI rad) into different
            # cells whose number is given by NUM_THETA_CELLS.
            interval = 2 * np.pi / self.dim[0]
            stack_num = 0
            while theta > interval:
                theta -= interval
                stack_num += 1
            if stack_num == self.NUM_THETA_CELLS:
                stack_num = 0
            return int(stack_num)
            
     def heuristic(self, x, y, goal):
            # TODO: implement a heuristic function.
            cost_manhattan_dist = abs(x - goal[0]) + abs(y - goal[1])
            return cost_manhattan_dist

- 최적의 경로 탐색이 특징인 A* 알고리즘과 다르게, Hybrid A* 알고리즘은 completeness, optimality를 포기하고 drivable path 를 찾을 수 있다.

- 자차의 조향각 (delta), 속도(speed)를 이용하여 자차가 이동할 위치를 구한다 -> "motion model"

- 차량이 이동할 다음 state (x y 좌표, delta 조향각)를 구하고, 각도를 정규화하여 map (grid) 내에 존재하는, 이동 가능한 state 만 입력하도록 한다. -> "expand"

- 시작점에서 목표점으로 도달할 때까지 반복하여 경로를 찾으며, map grid 내에 존재하는지와 장애물 & 중복 지점인지 확인하고 기록한다. -> "search" 

- 시작점에서 목표점까지의 cost 계산은 heuristic 알고리즘을 사용하였으며, heuristic cost 계산을 위해 Manhatten Distance 방법을 적용하였다. 

- 자차의 방향을 기준으로 입력 받은 조향각을 0~360 사이로 반환 -> "theta to stack num"

- 자차의 motion state (theta, speed)에 따라 결과가 달라진다.

- 1. speed = 1, num theta cells = 180


![image](https://user-images.githubusercontent.com/80089347/117464664-da236600-af8b-11eb-8a1e-5d16ec38130a.png)


- 2. speed = 1.5, num theta cells = 360


![image](https://user-images.githubusercontent.com/80089347/117464762-f1faea00-af8b-11eb-8f38-13046f4eb48f.png)



---

# Week 7 - Hybrid A* Algorithm & Trajectory Generation

---

[//]: # (Image References)
[has-example]: ./hybrid_a_star/has_example.png
[ptg-example]: ./PTG/ptg_example.png

## Assignment: Hybrid A* Algorithm

In directory [`./hybrid_a_star`](./hybrid_a_star), a simple test program for the hybrid A* algorithm is provided. Run the following command to test:

```
$ python main.py
```

The program consists of three modules:

* `main.py` defines the map, start configuration and end configuration. It instantiates a `HybridAStar` object and calls the search method to generate a motion plan.
* `hybrid_astar.py` implements the algorithm.
* `plot.py` provides an OpenCV-based visualization for the purpose of result monitoring.

You have to implement the following sections of code for the assignment:

* Trajectory generation: in the method `HybridAStar.expand()`, a simple one-point trajectory shall be generated based on a basic bicycle model. This is going to be used in expanding 3-D grid cells in the algorithm's search operation.
* Hybrid A* search algorithm: in the method `HybridAStar.search()`, after expanding the states reachable from the current configuration, the algorithm must process each state (i.e., determine the grid cell, check its validity, close the visited cell, and record the path. You will have to write code in the `for n in next_states:` loop.
* Discretization of heading: in the method `HybridAStar.theta_to_stack_num()`, you will write code to map the vehicle's orientation (theta) to a finite set of stack indices.
* Heuristic function: in the method `HybridAStar.heuristic()`, you define a heuristic function that will be used in determining the priority of grid cells to be expanded. For instance, the distance to the goal is a reasonable estimate of each cell's cost.

You are invited to tweak various parameters including the number of stacks (heading discretization granularity) and the vehicle's velocity. It will also be interesting to adjust the grid granularity of the map. The following figure illustrates an example output of the program with the default map given in `main.py` and `NUM_THETA_CELLS = 360` while the vehicle speed is set to 0.5.

![Example Output of the Hybrid A* Test Program][has-example]

---

## Experiment: Polynomial Trajectory Generation

In directory [`./PTG`](./PTG), a sample program is provided that tests polynomial trajectory generation. If you input the following command:

```
$ python evaluate_ptg.py
```

you will see an output such as the following figure.

![Example Output of the Polynomial Trajectory Generator][ptg-example]

Note that the above figure is an example, while the result you get will be different from run to run because of the program's random nature. The program generates a number of perturbed goal configurations, computes a jerk minimizing trajectory for each goal position, and then selects the one with the minimum cost as defined by the cost functions and their combination.

Your job in this experiment is:

1. to understand the polynomial trajectory generation by reading the code already implemented and in place; given a start configuration and a goal configuration, the algorithm computes coefficient values for a quintic polynomial that defines the jerk minimizing trajectory; and
2. to derive an appropriate set of weights applied to the cost functions; the mechanism to calculate the cost for a trajectory and selecting one with the minimum cost is the same as described in the previous (Week 6) lecture.

Experiment by tweaking the relative weight for each cost function. It will also be very interesting to define your own cost metric and implement it using the information associated with trajectories.
