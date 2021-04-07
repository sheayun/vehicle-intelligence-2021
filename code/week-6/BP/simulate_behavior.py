from road import Road
import time
import random

# impacts default behavior for most states
SPEED_LIMIT = 10

# all traffic in lane (besides ego) follow these speeds
# LANE_SPEEDS = [6, 7, 8]
LANE_SPEEDS = [6, 7, 8, 9]
# LANE_SPEEDS = [6, 7, 8, 9, 10]

# Number of available "cells" which should have traffic
TRAFFIC_DENSITY = 0.15
# TRAFFIC_DENSITY = 0.3

# At each timestep, ego can set acceleration to value between
# -MAX_ACCEL and MAX_ACCEL
MAX_ACCEL = 2

# s value and lane number of goal.
GOAL = (300, 0)

# These affect the visualization
FRAMES_PER_SECOND = 4
AMOUNT_OF_ROAD_VISIBLE = 40

def run_simulation(VISUALIZE=True):
    road = Road(
        SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS, AMOUNT_OF_ROAD_VISIBLE
    )
    road.populate_traffic()
    ego_config = {
        'speed_limit': SPEED_LIMIT,
        'num_lanes': len(LANE_SPEEDS),
        'goal': GOAL,
        'max_acceleration': MAX_ACCEL,
    }
    # Ego begins travelling at lane #2.
    road.add_ego(2, 0, ego_config)
    timestep = 0
    while road.get_ego().s <= GOAL[0]:
        timestep += 1
        if timestep > 150:  # simulation time limit
            if VISUALIZE:
                print("Taking too long to reach goal. Go faster!")
            break

        road.advance()
        if VISUALIZE:
            print(road)
            time.sleep(float(1.0) / FRAMES_PER_SECOND)

    # Completed to the goal distance
    ego = road.get_ego()
    if VISUALIZE:
        if ego.lane == GOAL[1]:
            print("You got to the goal in %d seconds!" % timestep)
        else:
            print("You missed the goal. "
                  "You are in lane %d instead of %d." % (ego.lane, GOAL[1]))
    return timestep, ego.lane


if __name__ == "__main__":
    # Adjust the following random seed to get different results.
    random.seed(1)
    timestep, lane = run_simulation(VISUALIZE=True)
