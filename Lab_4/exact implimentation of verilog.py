import numpy as np
import matplotlib.pyplot as plt
import pandas as pd  # <-- CSV handling
from numba import njit

# Chua's Circuit Constants
m0 = -1.143
m1 = -0.714
default_alpha = 15.6
default_beta = 28.0
dt = 1/1024
n_steps = 10_000

@njit
def chua_f(x):
    return m1 * x + 0.5 * (m0 - m1) * (abs(x + 1) - abs(x - 1))

@njit
def simulate_chua(alpha, beta, x0, y0, z0, dt, n_steps):
    x, y, z = x0, y0, z0
    for _ in range(n_steps):
        dx = alpha * (y - x - chua_f(x))
        dy = x - y + z
        dz = -beta * y
        x += dx * dt
        y += dy * dt
        z += dz * dt
    return x, y, z

def chua_rng_16bit(seed: int) -> int:
    rng = np.random.default_rng(seed)
    x0 = rng.uniform(-1, 1)
    y0 = rng.uniform(-1, 1)
    z0 = rng.uniform(-1, 1)
    alpha = default_alpha + rng.uniform(-0.5, 0.5)
    beta = default_beta

    x, _, _ = simulate_chua(alpha, beta, x0, y0, z0, dt, n_steps)

    # Immediate check!
    if np.isnan(x) or np.isinf(x):
        print(f"[Warning] Simulation unstable for seed {seed}, forcing x=0.0")
        x = 0.0

    # Now safe to proceed
    x_clipped = np.clip(x, -10, 10)
    normalized = (x_clipped + 10) / 20

    # Extra safety: if normalized somehow NaN
    if np.isnan(normalized) or np.isinf(normalized):
        normalized = 0.5  # mid-range default

    return int(normalized * 65535)


# --- Example usage ---

your_seed = int(input("Enter your seed: "))
rand_value = chua_rng_16bit(your_seed)
print(f"16-bit random number for seed {your_seed}: {rand_value}")

# --- Now let's plot over multiple seeds ---

# Define a range of seeds
seed_values = np.arange(0, 200)  # Seeds from 0 to 199
rand_outputs = []

for seed in seed_values:
    rand_outputs.append(chua_rng_16bit(seed))

rand_outputs = np.array(rand_outputs)

# --- Plotting ---

plt.figure(figsize=(10, 5))
plt.plot(seed_values, rand_outputs, marker='o', markersize=3, linestyle='-', linewidth=1)
plt.title("Chua RNG Output vs Seed")
plt.xlabel("Seed Value")
plt.ylabel("16-bit Random Output")
plt.grid(True)
plt.tight_layout()
plt.show()

# Optional: Histogram of outputs to see distribution
plt.figure(figsize=(6,4))
plt.hist(rand_outputs, bins=50, edgecolor='black', density=True)
plt.title("Histogram of Chua RNG 16-bit outputs")
plt.xlabel("Random Value")
plt.ylabel("Density")
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Save to CSV ---
df = pd.DataFrame({
    'seed': seed_values,
    'random_output': rand_outputs
})

csv_filename = "chua_rng_outputs.csv"
df.to_csv(csv_filename, index=False)
print(f"Data saved to {csv_filename}")
