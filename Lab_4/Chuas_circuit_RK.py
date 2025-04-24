import numpy as np
import matplotlib.pyplot as plt
from numba import njit

# Parameters
alpha = 15.6
beta = 28.0
m0 = -1.143
m1 = -0.714
dt = 0.01
n_steps = 5_000_000

@njit
def chua_f(x):
    return m1 * x + 0.5 * (m0 - m1) * (abs(x + 1) - abs(x - 1))

@njit
def simulate_chua(n_steps, dt, alpha, beta):
    x, y, z = 0.1, 0.0, 0.0
    xs = np.empty(n_steps, dtype=np.float32)
    ys = np.empty(n_steps, dtype=np.float32)
    zs = np.empty(n_steps, dtype=np.float32)

    for i in range(n_steps):
        dx = alpha * (y - x - chua_f(x))
        dy = x - y + z
        dz = -beta * y

        x += dx * dt
        y += dy * dt
        z += dz * dt

        xs[i] = x
        ys[i] = y
        zs[i] = z

    return xs, ys, zs

# Run the simulation (compiled version)
xs, ys, zs = simulate_chua(n_steps, dt, alpha, beta)

# Plot a downsampled version
plt.figure(figsize=(8, 6))
plt.plot(xs[::100], zs[::100], linewidth=0.5)
plt.xlabel("x")
plt.ylabel("z")
plt.title("Chua's Circuit Phase Portrait (Numba Accelerated)")
plt.grid(True)
plt.tight_layout()
plt.show()


