import numpy as np

data = np.array([114.97, 120.95, 137.33, 127.58, 123.63])

mean  = data.mean()
std = data.std()

print(f"Mean: {mean}, std: {std}")

#95% ci = 1.645