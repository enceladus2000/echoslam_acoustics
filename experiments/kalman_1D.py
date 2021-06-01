import math
import matplotlib.pyplot as plt
import numpy as np

# Parameters Test
gt = 72         # Ground Truth
mea_i = 75      # Initial Measurement
E_mea = 4       # Error in measurement
E_est_i = 2     # Initial estimate in error
est_i = 68      # Initial estimate
max_time = 20   # Time in seconds
frequency = 1   # Frequency of sampling in Hz
samples = max_time * frequency

# Initial calculations
KG = E_est_i/(E_est_i + E_mea)
est = est_i + KG * (mea_i - est_i)
E_est = (1 - KG) * E_est_i

# Arrays
measurements = [mea_i]
estimates = [est]
t = [0]

# Further Sampling iterations
for i in range(samples):
    mea = np.random.normal(mea_i,E_mea)

    KG = E_est/(E_est + E_mea)
    est = est + KG * (mea - est)
    E_est = (1 - KG) * E_est

    measurements.append(mea)
    estimates.append(est)
    t.append(i+1)

measurements = np.array(measurements)
estimates = np.array(estimates)
time = np.array(t)


# Plot
plt.hlines(gt,0,max_time,'r')       # Ground truth
plt.plot(time,measurements,'b')     # Measurements
plt.plot(time,estimates,'g')        # Estimates
plt.legend(["Measurements", "Estimates", "Ground Truth"])
plt.ylabel('Something')
plt.xlabel('Samples (n)')
plt.axis([0,20,60,90])
plt.show()