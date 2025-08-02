import pandas as pd
import numpy as np
import ast
import matplotlib.pyplot as plt

csv_path = "output.csv"
df = pd.read_csv(csv_path)

def flatten_val(val):
    if isinstance(val, str) and val.startswith('['):
        try:
            parsed = ast.literal_eval(val)
            if isinstance(parsed, list) and len(parsed) == 1:
                return float(parsed[0])
        except:
            return np.nan
    try:
        return float(val)
    except:
        return np.nan

# Convert time and velocities
df['%time'] = pd.to_numeric(df['%time'], errors='coerce')
df['vx'] = df['field.twist.linear.x'].map(flatten_val)
df['vy'] = df['field.twist.linear.y'].map(flatten_val)

# Filter for time between 4.8 and 5.2 seconds
print("Min %time:", df['%time'].min())
print("Max %time:", df['%time'].max())
time_min, time_max = 4.8, 5.2
window_df = df[(df['%time'] >= time_min) & (df['%time'] <= time_max)]
print(window_df.head())
# Filter out rows with NaN in vx or vy
window_df = window_df.dropna(subset=['vx', 'vy'])

vx = window_df['vx'].to_numpy()
vy = window_df['vy'].to_numpy()

# Compute mean velocities
mean_vx = np.mean(vx)
mean_vy = np.mean(vy)

# Compute theta (yaw offset)
theta = np.arctan2(mean_vy, mean_vx)
print(f"Estimated DVL yaw offset (radians): {theta:.4f}")
print(f"Estimated DVL yaw offset (degrees): {np.degrees(theta):.2f}")

# Optional: plot before and after rotation for this window
cos_t = np.cos(-theta)
sin_t = np.sin(-theta)

vx_body = cos_t * vx + sin_t * vy
vy_body = cos_t * vy - sin_t * vx

plt.figure(figsize=(12,5))
plt.subplot(1,2,1)
plt.scatter(vx, vy, s=10)
plt.xlabel('v_x (DVL frame)')
plt.ylabel('v_y (DVL frame)')
plt.title(f'Original velocities [{time_min}s - {time_max}s]')
plt.axis('equal')
plt.grid(True)

plt.subplot(1,2,2)
plt.scatter(vx_body, vy_body, s=10, color='orange')
plt.xlabel('v_x (Body frame)')
plt.ylabel('v_y (Body frame)')
plt.title('Rotated velocities')
plt.axis('equal')
plt.grid(True)

plt.tight_layout()
plt.show()
