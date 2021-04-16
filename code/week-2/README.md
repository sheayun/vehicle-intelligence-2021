# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

[HW_Week2]


[1] Motion model (assuming 1-D Gaussian dist)
def motion_model(position, mov, priors, map_size, stdev):
  
    position_prob = 0.0

    for i in range(map_size):
        position_prob += norm_pdf(position-i,stdev, mov)*priors[i]
   
    return position_prob
    
- Motion Model 은 차량이 이동할 수 있는 모든 직전 위치의 확률과 이동에 따른 현재 위치를 예측하여 확률로 나타낸다.
- Position Prob 는 차량 위치를 예측한 확률로, 차량이 존재할 수 있는 모든 이전 위치의 확률을 Priors 로 나타낸다
또한 이전 위치에서 현재 차량이 존재하는 위치로 이동할 수 있는 확률은 Position 으로 , 직전 위치 확률 * 현재 위치로 올 확률로 계산한다.    

[2] Observation model (assuming independent Gaussian)
def observation_model(landmarks, observations, pseudo_ranges, stdev):
  
    distance_prob = 1.0
    
    if len(observations) == len(pseudo_ranges):
        for i in range(len(pseudo_ranges)):
            distance_prob *= norm_pdf(pseudo_ranges[i],observations[i],1)
    else : distance_prob = 0
            
    return distance_prob
    
- Observations Model 은 차량 주변의 나무(LandMark)를 관측할 수 있지만, 불확실성을 가지고 있다. 
관측한 나무 위치 값을 기준으로 차량이 존재할 수 있는 위치에 대한 신뢰도를 높인다.

- 나무가 존재하여도 관측 값이 0이거나, 나무 수보다 많게 관측되면 사용할 수 없는 값으로 처리한다.
신뢰할만한 관측 결과는 (관측된 나무 위치 * 현재 차량 위치에서 나무가 관측될 확률)로 나타낼 수 있다. 

- 정확도가 점차 커지는 과정은 관측된 수만큼 반복하여 차량에서 관측한 나무 사이의 거리와 나무가 그 거리에서 보일 확률을 곱하여 누적한다. 
위와 같은 반복 작업으로 계산된 위치에 대한 확률과 센서 측정에 대한 확률을 이용하여 현재 차량이 위치할 수 있는 확률을 나타낸다. 
    

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.
    
