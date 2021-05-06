# Week 3 - Kalman Filters, EKF and Sensor Fusion

### Assignment

Complete the implementation of EKF with sensor fusion by writing the function `update_ekf()` in the module `kalman_filter`. Details are given in class and instructions are included in comments.

[HW_week3]
Update Extended Kalman filter

- EKF는 기존의 Kalman Filter와 유사하나 선형화하는 기준점을 계속 갱신한다는 특징을 가지고 있다.
EKF 업데이트는 레이더 관측을 활용한다.         
        
       def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements       
        # 1. Compute Jacobian Matrix H_j
        px, py, vx, vy = self.x
        H_j = Jacobian(self.x)
         
- 자코비안(Jacobian) 행렬은 원소들이 모두 1차 미분 계수로 구성되어 있으며, 미소 영역에서 ‘비선형 변환’을 ‘선형 변환으로 근사화 시키는 것이 특징이다.
Jacobian 함수를 이용하여 상태변수 Jacobian 행렬을 계산한다.        
        
        # 2. Calculate S = H_j * P' * H_j^T + R
        S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R
      
- Kalman Gain "k" 를 구하기 위해 (자코비안 행렬 * 예측 공분산 P * 측정 에러 공분산 R)을 계산한다.      
        
        # 3. Calculate Kalman gain K = H_j * P' * H_j^T + R
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))
        
- Kalman Gain "K" 계산      
        
        # 4. Estimate y = z - h(x')
        y = z - [sqrt(px*px+py*py), atan2(py,px), (px*vx+py*vy)/sqrt(px*px+py*py)]
        
- 레이더 관측 변수 "z" 와 관측 상태변수 "y" 차이로 에러를 구한다.
        
        # 5. Normalize phi so that it is between -PI and +PI
        while (y[1] > pi): y[1] -= 2*pi
        while (y[1] < -pi): y[1] += 2*pi
            
- 관측 변수의 단위는 라디안(rad)이고, 모든 각도를 pi : -pi 사이의 값을 가질 수 있다.
위 각도 범위를 벗어나지 않도록 조건을 생성한다.          
            
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, H_j), self.P)

- 예측 상태변수 "x" 와 관측 에러 "y", Kalman Gain "K" 를 활용하여 새로운 예측 상태변수 "x" 를 구할 수 있다.
새로운 공분산 "P" 도 자코비안 "H_j" 와 Kalman Gain "K" 를 활용하여 계산할 수 있다.

- 측정 에러와 새로운 예측을 반복하여 매 Sample 마다 위치 예측을 갱신한다.

---

[//]: # (Image References)
[kalman-result]: ./kalman_filter/graph.png
[EKF-results]: ./EKF/plot.png

## Kalman Filter Example

In directory [`./kalman_filter`](./kalman_filter), a sample program for a small-scale demonstration of a Kalman filter is provided. Run the following command to test:

```
$ python testKalman.py
```

This program consists of four modules:

* `testKalman.py` is the module you want to run; it initializes a simple Kalman filter and estimates the position and velocity of an object that is assumed to move at a constant speed (but with measurement error).
* `kalman.py` implements a basic Kalman fitler as described in class.
* `plot.py` generates a plot in the format shown below.
* `data.py` provides measurement and ground truth data used in the example.

The result of running this program with test input data is illustrated below:

![Testing of Kalman Filter Example][kalman-result]

Interpretation of the above results is given in the lecture.

In addition, you can run `inputgen.py` to generate your own sample data. It will be interesting to experiment with a number of data sets with different characteristics (mainly in terms of variance, i.e., noise, involved in control and measurement).

---

## Assignment - EFK & Sensor Fusion Example

In directory [`./EKF`](./EKF), template code is provided for a simple implementation of EKF (extended Kalman filter) with sensor fusion. Run the following command to test:

```
$ python run.py
```

The program consists of five modules:

* `run.py` is the modele you want to run. It reads the input data from a text file ([data.txt](./EKF/data.txt)) and feed them to the filter; after execution summarizes the result using a 2D plot.
* `sensor_fusion.py` processees measurements by (1) adjusting the state transition matrix according to the time elapsed since the last measuremenet, and (2) setting up the process noise covariance matrix for prediction; selectively calls updated based on the measurement type (lidar or radar).
* `kalman_filter.py` implements prediction and update algorithm for EKF. All the other parts are already written, while completing `update_ekf()` is left for assignment. See below.
* `tools.py` provides a function `Jacobian()` to calculate the Jacobian matrix needed in our filter's update algorithm.
*  `plot.py` creates a 2D plot comparing the ground truth against our estimation. The following figure illustrates an example:

![Testing of EKF with Sensor Fusion][EKF-results]
