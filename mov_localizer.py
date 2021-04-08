from helper import norm_pdf

# Initialize prior probabilities
# taking into account that the vehicle is initially parked
# around one of the landmarks and we do not know which.
def initialize_priors(map_size, landmarks, stdev):
    # Set all probabilities to zero initially.
    priors = [0.0] * map_size
    # Initialize prior distribution assuming the vehicle is at
    # landmark +/- 1.0 meters * stdev.
    positions = []
    for p in landmarks:
        start = int(p - stdev) - 1
        if start < p:
            start += 1
        c = 0
        while start + c <= p + stdev:
            # Gather positions to set initial probability.
            positions.append(start + c)
            c += 1
    # Calculate actual probability to be uniformly distributed.
    prob = 1.0 / len(positions)
    # Set the probability to each position.
    for p in positions:
        priors[p] += prob
    return priors

# Estimate pseudo range determined according to the
# given pseudo position.
def estimate_pseudo_range(landmarks, p):
    pseudo_ranges = []
    # Loop over each landmark and estimate pseudo ranges
    for landmark in landmarks:
        dist = landmark - p
        # Consider only those landmarks ahead of the vehicle.
        if dist > 0:
            pseudo_ranges.append(dist)
    return pseudo_ranges

# Motion model (assuming 1-D Gaussian dist)
def motion_model(position, mov, priors, map_size, stdev):
    # Initialize the position's probability to zero.
    position_prob = 0.0

    for i in range(map_size):
        position_prob += norm_pdf(position-i,stdev, mov)*priors[i]
        
    # TODO: Loop over state space for all possible prior positions,
    # calculate the probability (using norm_pdf) of the vehicle
    # moving to the current position from that prior.
    # Multiply this probability to the prior probability of
    # the vehicle "was" at that prior position.
    return position_prob

# Observation model (assuming independent Gaussian)
def observation_model(landmarks, observations, pseudo_ranges, stdev):
    # Initialize the measurement's probability to one.
    distance_prob = 1.0
    
    if len(observations) == len(pseudo_ranges):
        for i in range(len(pseudo_ranges)):
            distance_prob *= norm_pdf(pseudo_ranges[i],observations[i],1)
    else : distance_prob = 0
            
    # TODO: Calculate the observation model probability as follows:
    # (1) If we have no observations, we do not have any probability.
    # (2) Having more observations than the pseudo range indicates that
    #     this observation is not possible at all.
    # (3) Otherwise, the probability of this "observation" is the product of
    #     probability of observing each landmark at that distance, where
    #     that probability follows N(d, mu, sig) with
    #     d: observation distance
    #     mu: expected mean distance, given by pseudo_ranges
    #     sig: squared standard deviation of measurement
    return distance_prob

# Normalize a probability distribution so that the sum equals 1.0.
def normalize_distribution(prob_dist):
    normalized = [0.0] * len(prob_dist)
    total = sum(prob_dist)
    for i in range(len(prob_dist)):
        if (total != 0.0):
            normalized[i] = prob_dist[i] / total
    return normalized
