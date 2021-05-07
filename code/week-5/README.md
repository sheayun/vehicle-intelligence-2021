## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in policy.py. A template code is given by assignment.py.

[HW_week5]

def optimum_policy_2D(grid, init, goal, cost):
    # Initialize the value function with (infeasibly) high costs.
    value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
    # Initialize the policy function with negative (unused) values.
    policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
    # Final path policy will be in 2D, instead of 3D.
    policy2D = np.full(grid.shape, ' ')

    # Apply dynamic programming with the flag change.
    change = True
    while change:
        change = False
        # This will provide a useful iterator for the state space.
        p = itertools.product(
            range(grid.shape[0]),
            range(grid.shape[1]),
            range(len(forward))
        )
        # Compute the value function for each state and
        # update policy function accordingly.
        for y, x, t in p:
            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
                # TODO: implement code.
                value[(t,y,x)] = 0;
                policy[(t,y,x)] = -999
                change = True
                
            # Try to use simple arithmetic to capture state transitions.
            elif grid[(y, x)] == 0:
                # TODO: implement code.
                for i, act in enumerate(action):
                    direction = (t+act) % 4
                    y2, x2 = y + forward[direction][0], x + forward[direction][1]
                    
                    if 0 <= y2 < grid.shape[0] \
                        and 0 <= x2 < grid.shape[1] \
                        and grid[(y2, x2)] == 0:        
                            val = value[(direction, y2, x2)] + cost[i]
                            
                            if val < value[(t, y, x)]:
                                value[(t, y, x)] = val
                                policy[(t, y, x)] = act
                                change = True
				
    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
    # TODO: implement code.
    y,x,o = init
    
    policy_start = policy[(o, y, x)]
    for i in range(len(action)):
        if policy_start == action[i]:
            policy_ID = action_name[i]

    policy2D[(y, x)] = policy_ID
    while policy[(o, y, x)] != -999:
        if policy[(o, y, x)] == action[0]:
            o2 = (o - 1) % 4  # turn left
        elif policy[(o, y, x)] == action[1]:
            o2 = o  # go straight
        elif policy[(o, y, x)] == action[2]:
            o2 = (o + 1) % 4  # turn right

        y, x = y + forward[o2][0], x + forward[o2][1]
        o = o2

        policy_temp = policy[(o,y,x)]
        if policy_temp == -999:
            policy_name = "*"
        else:
            for i in range(len(action)):
                if policy_temp == action[i]:
                    policy_name = action_name[i]

        policy2D[(y,x)] = policy_name


- "경로 탐색 알고리즘"

- 차량 모델에서 Heading Angle 을 고려하여 거동에 따른 Cost를 계산하여 최적화된 경로를 탐색한다.

- 차량의 방향 "t" 이용하여 Optimal_Policy_2D 초기 위치인 start 위치에 차량의 이동 방향 정보를 갱신한다.

- 반복하여 이동할 지역을 탐색하다 목적지에 도착하면 탐색을 종료하기 위해 value = 0, policy = -999 로 설정.

- 이동이 가능한 지역인지 Map 범위 안에 있는지 확인한다.

- 차량의 action은 직진,좌회전,후진,우회전(up / left / down / right) 을 의미하며 
4가지 거동에 대한 방향은 2차원 x,y 위에 존재한다. -> "t"

- 차량 action에 따른 이동에 대한 정보는 현재 상태와 방향에서 이동된 상태와 방향으로
갱신된 action 정보를 나타낸다 -> "act"

- 갱신된 상태와 방향이 Map grid 내의 지점이면 action에 대한 cost를 부여해 value 값 에 더해주고, 
기존의 value 와 비교하여 최적의 action으로 갱신한다.  

- 시작점에서 출발하여 차량 거동에따라 갱신된 action 정보를 policy 에 저장 하는 것을 반복하여 최적 경로를 탐색한다.
-> "Policy_ID"

- 목표지점에 도달하지 못하거나, map grid 를 벗어나게 된다면 경로 탐색은 실패한다.

- 좌회전 cost에 따른 결과 비교 (가중치를 크게 주면 교차로에서 직진 후 우회전으로 돌아가는 것을 확인할 수 있다.)
- cost (2, 1, 10) <-> (2, 1, 100)


![image](https://user-images.githubusercontent.com/80089347/117440459-07155000-af6f-11eb-94bd-fe18fd04de7f.png)



# Week 5 - Path Planning & the A* Algorithm

---

## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.
