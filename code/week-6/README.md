## HW_week 6 GNB & BP

# Gaussian Naive Bayes Classifier
---

	def train(self, X, Y):
		    self.X_arr, self.Y_arr = np.array(X), np.array(Y)
		    unique, cnt = np.unique(Y, return_counts=True)
		    self.priors = cnt / len(Y)

		    self.Means = np.array([self.X_arr[np.where(self.Y_arr==i)].mean(axis=0) for i in self.classes])
		    self.Standevi = np.array([self.X_arr[np.where(self.Y_arr==i)].std(axis=0) for i in self.classes])

		    return self

	def predict(self, observation):
		    probas = np.zeros(len(self.classes))
		    # Calculate Gaussian probability for each variable based on the
		    # mean and standard deviation calculated in the training process.
		    for i in range(len(self.classes)):
			probability = 1
			for k in range(self.X_arr.shape[1]):
			    # Multiply all the probabilities for variables, and then
			    # normalize them to get conditional probabilities.
			    probability *= gaussian_prob(observation[k], self.Means[i][k], self.Standevi[i][k])

			probas[i] = probability
		    nmz = probas / probas.sum()

		    # Return the label for the highest conditional probability.
		    return self.classes[nmz.argmax(axis=0)]

- Gaussian Naive Bayes Classifier 는 차량의 궤적 (좌회전 / 직진 / 우회전) 에 대한 데이터를 수집하고 평균과 표준편차를 계산하여 관측을 통해 확률이 높은 주행 경로를 예측한다.

- 수집된 데이터를 분류하여 사전확률을 구하고 주행 state (s / d / sd_ot / d_dot)에 대한 평균과 표준편차를 계산한다.

- 관측 데이터의 확률을 가우시안으로 계산하고 사전확률을 곱한다.

- 주행 state에 대한 평균, 표준편차와 관측 데이터를 이용하여 가장 높은 확률의 주행경로를 예측한다. 

- 100%의 정확도 기준으로 84%의 확률을 결과로 나타낸다.


![image](https://user-images.githubusercontent.com/80089347/117452276-86f6e680-af7e-11eb-9b7e-702cf88a0386.png)


# Behaviour Planning
---

	def choose_next_state(self, predictions):
		'''
		Implement the transition function code for the vehicle's
		behaviour planning finite state machine, which operates based on
		the cost function (defined in a separate module cost_functions.py).
		INPUTS: A predictions dictionary with vehicle id keys and predicted
		    vehicle trajectories as values. Trajectories are a list of
		    Vehicle objects representing the vehicle at the current timestep
		    and one timestep in the future.
		OUTPUT: The the best (lowest cost) trajectory corresponding to
		    the next ego vehicle state.
		Functions that will be useful:
		1. successor_states():
		    Returns a vector of possible successor states
		    for the finite state machine.
		2. generate_trajectory(self, state, predictions):
		    Returns a vector of Vehicle objects representing a
		    vehicle trajectory, given a state and predictions.
		    Note that trajectories might be empty if no possible trajectory
		    exists for the state; for example, if the state is LCR, but a
		    vehicle is occupying the space to the ego vehicle's right,
		    then there is no possible trajectory without first
		    transitioning to another state.
		3. calculate_cost(vehicle, trajectory, predictions):
		    Imported from cost_functions.py, computes the cost for
		    a trajectory.
		'''

		# TODO: implement state transition function based on the cost
		#       associated with each transition.


		possible_successor_states = self.successor_states()
		costs = []
		for state in possible_successor_states:
		    print("possibles:", state)
		    trajectory = self.generate_trajectory(state, predictions)
		    print("trajectory state:", trajectory[1].state)
		    cost_for_state = calculate_cost(self, trajectory, predictions)
		    costs.append({'state': state, 'cost': cost_for_state})

		good_state = None
		min_cost = 9999999
		for c in costs:
		    if c['cost'] < min_cost:
			min_cost = c['cost']
			good_state = c['state']

		next_trajectory = self.generate_trajectory(good_state, predictions)
		# Note that the return value is a trajectory, where a trajectory
		# is a list of Vehicle objects with two elements.
		return next_trajectory


	def goal_distance_cost(vehicle, trajectory, predictions, data):
	    '''
	    Cost increases based on distance of intended lane (for planning a
	    lane change) and final lane of a trajectory.
	    Cost of being out of goal lane also becomes larger as vehicle approaches
	    the goal distance.
	    '''
	    intended_lane_distance = vehicle.goal_lane - data[0]
	    final_lane_distance = vehicle.goal_lane - data[1]
	    goal_distance = data[2]

	    if goal_distance / vehicle.goal_s > 0.4:
		cost = 0.0
	    elif goal_distance > 0:
		cost = 1 - exp((intended_lane_distance + final_lane_distance) / (goal_distance))
	    else:
		cost = 1
	    print("goal_distance_cost:", cost)
	    return cost

	def inefficiency_cost(vehicle, trajectory, predictions, data):
	    '''
	    Cost becomes higher for trajectories with intended lane and final lane
	    that have slower traffic.
	    '''
	    intented_lane = data[0]
	    final_lane = data[1]
	    goal_distance = data[2]

	    if goal_distance / vehicle.goal_s > 0.4:
		cost = exp(-(intented_lane + final_lane))
	    else:
		cost = 1 - exp(-(intented_lane + final_lane))
	    # print(cost)
	    return cost
    
    
- 목표 차로와 차량의 속도에 따라 cost를 부여하여 현재 궤적을 구한다.
 
- 목표 차로에 도달하기 위해 차로 변경을 하며 목표지점에 가까워 질수록 cost가 증가한다.  

- 현재 자차의 위치에 대한 상태 정보 (KL/ PLCL/ PLCR/ LCL/ LCR) 을 어떻게 정의할 것인지 결정하고, 
현재 상태를 보고 이동가능한 차로가 있는지 이동 궤적을 계산한다. 계산된 궤적 중 최소 cost를 선택한다 -> "choose next state"

- 목표 차로와 자차가 주해중인 차로 사이의 거리를 계산하여 3개의 weight를 산출하고  cost를 부여한다. 
(intended / final lane) -> "goal distance cost"

- 속도가 낮으면 목표 도달까지 시간이 지연되기 때문에 cost 를 더 부여하며, 
최적의 경로 (최소 cost)는 속도가 가장 높은 차로로 주행하다가 목적지 인근에서 차로 변경을 하는 것이다.

- 자차는 차로변경을 이용한 주행으로 목표 차로까지 도달하는데 33초가 소요되었다.



![image](https://user-images.githubusercontent.com/80089347/117452946-519ec880-af7f-11eb-854c-d5af6a303f59.png)





# Week 6 - Prediction & Behaviour Planning
---

## Assignment #1

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.

Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.


---

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

* determine which state transitions are possible from the current state (`successor_states()` function in the same module will be helpful),
* calculate cost for each state transition using the trajectory generated for each behaviour, and
* select the minimum cost trajectory and return it.

Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.
