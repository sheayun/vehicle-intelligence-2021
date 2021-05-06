# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

[HW_week4] 
[1] Update Weights

 # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        import math
       # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range)
        
        for p in self.particles:
            visible_landmarks = []
            for id, landmark in map_landmarks.items():
            
                if distance(p, landmark) < sensor_range:
                    visible = {'x': landmark['x'], 
                               'y': landmark['y'], 'id': id}
                    visible_landmarks.append(visible)
                    
- distance(p, landmark) 함수를 이용하여 map에 표시된 랜드마크와 파티클 사이의 거리를 계산한다.
- 계측된 거리가 Sensor range 보다 작으면 visible 함수에 랜드마크 id와 x, y 값을 저장한다.                      
  
       # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
        
            transformed_obs = []
            for obs in observations:
                t_observ = {}
                t_observ['x'] = p['x'] + (obs['x'] * np.cos(p['t'])) - \ (obs['y'] * np.sin(p['t']))
                t_observ['y'] = p['y'] + (obs['x'] * np.sin(p['t'])) + \ (obs['y'] * np.cos(p['t']))
                transformed_obs.append(t_observ)

            if len(visible_landmarks) == 0:
                continue

- 계측된 랜드마크의 x, y 좌표를 Map 기준의 global 좌표로 변환시킨다.

       # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
        
             assoc_landmarks = self.associate(visible_landmarks, transformed_obs)
            p['assoc'] = [landmark['id'] for landmark in assoc_landmarks]
            
- Sensor range 내의 랜드마크와 global 좌표로 변환된 랜드마크를 매칭 시킨다.
- 가장 가까운 랜드마크를 반환하기 위해 랜드마크의 id 를 "assoc" 의 값으로 입력한다.            
            
       # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        
            for t, a in zip(transformed_obs, assoc_landmarks):
            
                Gaussian_x = norm_pdf(t['x'], a['x'], std_landmark_x)
                Gaussian_y = norm_pdf(t['y'], a['y'], std_landmark_y)
                
                Gaussian = Gaussian_x * Gaussian_y

- transformed 랜드마크가 해당 위치에 존재할 확률은 multi-variate Gaussian distribution 으로 계산 가능하며,              
assoc의 랜드마크 좌표 x, y가 표준 편차이여야 한다.             


      # 5. Update the particle's weight by the calculated probability.
        
                 p['w'] *= Gaussian
                
- transformed 의 랜드마크에 대한 확률 값을 Gaussian 으로 구하여 최종 값이 Particle 의 Weight로 곱해진다.
- 랜드마크 매칭 및 파티클 확률을 가우시안 분포로 계산하고, Particle weight 를 업데이트한다.   

[2] Resample

 # Resample particles with replacement with probability proportional to
    #   their weights.
    
    def resample(self):
        
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        
        weights = [p['w'] for p in self.particles.copy()] 
        resampled_particles = []
        positions = (np.arange(len(weights)) + np.random.random()) / len(weights)
    
        idx = np.zeros(len(weights), 'i')
        cum_sum = np.cumsum(weights)
        
        i, j = 0, 0
        while i < len(weights) and j< len(weights) :
            if positions[i] < cum_sum[j]:
                idx[i] = j
                i += 1
            else:
                j += 1
                
- 파티클 weight 에 따라 resampling 되며 가중치가 크면 여러 번, 가중치가 작으면 클 때보다는 작게 sampling 된다.
- weight idx 함수를 이용하여 다음 주기의 Particle 에 weihted sampling 하게 된다.

        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        #    Finally, self.particles shall contain the newly drawn set of
        #    particles.
        
       resampled_particles.append(self.particles[idx[i]])
        self.particles = resampled_particles
        
        # return

- resampling 된 파티클을 복사하여 재입력한다.


You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.
