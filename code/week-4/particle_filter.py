import numpy as np
from helpers import *

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    # Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        for p in self.particles:
            visible_landmarks = []
            for id, landmark in map_landmarks.items():
                if distance(p, landmark) < sensor_range:
                    visible = {'x': landmark['x'], 'y': landmark['y'], 'id': id}
                    visible_landmarks.append(visible)

        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
            transformed_obs = []
            for obs in observations:
                t_observ = {}
                t_observ['x'] = p['x'] + (obs['x'] * np.cos(p['t'])) - \
                                        (obs['y'] * np.sin(p['t']))
                t_observ['y'] = p['y'] + (obs['x'] * np.sin(p['t'])) + \
                                        (obs['y'] * np.cos(p['t']))
                transformed_obs.append(t_observ)

            if len(visible_landmarks) == 0:
                continue

        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
            assoc_landmarks = self.associate(visible_landmarks, transformed_obs)
            p['assoc'] = [landmark['id'] for landmark in assoc_landmarks]

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

        # 5. Update the particle's weight by the calculated probability.
                p['w'] *= Gaussian
            
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
        
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        #    Finally, self.particles shall contain the newly drawn set of
        #    particles.
        
        resampled_particles.append(self.particles[idx[i]])
        self.particles = resampled_particles
        
        # return

    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
