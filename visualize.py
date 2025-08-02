import pandas as pd
import matplotlib.pyplot as plt
import ast
import numpy as np

csv_path = "output.csv"

df = pd.read_csv(csv_path)

# Use %time as x-axis, convert to numeric and drop NaNs
df['%time'] = pd.to_numeric(df['%time'], errors='coerce')

def flatten_val(val):
    if isinstance(val, str) and val.startswith('['):
        try:
            parsed = ast.literal_eval(val)
            if isinstance(parsed, list) and len(parsed) == 1:
                return float(parsed[0])
        except:
            return None
    try:
        return float(val)
    except:
        return None

target_cols = [
    'field.twist.linear.x',
    'field.twist.linear.y',
    'field.twist.linear.z',
    'field.twist.angular.x',
    'field.twist.angular.y',
    'field.twist.angular.z'
]

plt.figure(figsize=(12, 6))

for col in target_cols:
    if col in df.columns:
        y = df[col].map(flatten_val)
        
        # Drop NaNs for both time and y together to keep them aligned
        valid_mask = (~df['%time'].isna()) & (~y.isna())
        x = df.loc[valid_mask, '%time'].to_numpy()
        y_vals = y.loc[valid_mask].to_numpy()
        
        plt.plot(x, y_vals, label=col)

plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("Twist Linear & Angular Velocities")
plt.legend()
plt.grid(True)
plt.show()
