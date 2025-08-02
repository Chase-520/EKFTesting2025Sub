import pandas as pd
import matplotlib.pyplot as plt
import os
import ast

csv_path = "output.csv"
plot_dir = "plots"
os.makedirs(plot_dir, exist_ok=True)

df = pd.read_csv(csv_path)

# Use %time as x-axis
df['%time'] = pd.to_numeric(df['%time'], errors='coerce')
time_col = '%time'

# Flatten stringified lists like "[0.1]"
def flatten_column(series):
    def convert(val):
        if isinstance(val, str) and val.startswith('['):
            try:
                parsed = ast.literal_eval(val)
                if isinstance(parsed, list) and len(parsed) == 1:
                    return parsed[0]
            except:
                return None
        return val
    return series.apply(convert)

# Target fields
target_cols = [
    'field.twist.linear.x',
    'field.twist.linear.y',
    'field.twist.linear.z',
    'field.twist.angular.x',
    'field.twist.angular.y',
    'field.twist.angular.z'
]

# Process and plot
for col in target_cols:
    if col not in df.columns:
        print(f"  ⛔ Missing column: {col}")
        continue

    try:
        series = flatten_column(df[col])
        y = pd.to_numeric(series, errors='coerce')
        x = df[time_col]
        mask = (~x.isna()) & (~y.isna())
        x = x[mask]
        y = y[mask]

        if len(x) == 0:
            print(f"  ⚠️ No valid data for: {col}")
            continue

        plt.figure(figsize=(10, 4))
        plt.plot(x, y, label=col)
        plt.xlabel("Time (s)")
        plt.ylabel(col)
        plt.title(col)
        plt.grid(True)
        plt.tight_layout()

        out_file = os.path.join(plot_dir, col.replace('.', '_') + ".png")
        plt.savefig(out_file)
        plt.close()
        print(f"  ✅ Saved: {out_file}")

    except Exception as e:
        print(f"  ⚠️ Failed to plot {col}: {e}")

print("✅ All done.")
