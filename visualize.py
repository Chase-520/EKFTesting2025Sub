import pandas as pd
import matplotlib.pyplot as plt
import os

# Path to the CSV file
csv_path = "output.csv"

# Output folder for plots
plot_dir = "plots"
os.makedirs(plot_dir, exist_ok=True)

# Load CSV
df = pd.read_csv(csv_path)

# Try to detect a time column
time_col = None
for col in df.columns:
    if 'time' in col.lower():
        time_col = col
        break

if time_col is None:
    print("⚠️ No time column found. Using row index as x-axis.")
    df['index'] = df.index
    time_col = 'index'

# Ensure time column is numeric
df[time_col] = pd.to_numeric(df[time_col], errors='coerce')

# Get numeric columns (excluding time)
numeric_cols = df.select_dtypes(include='number').columns.tolist()
numeric_cols = [col for col in numeric_cols if col != time_col]

if not numeric_cols:
    print("❌ No numeric columns to plot.")
    exit()

# Plot each numeric column
for col in numeric_cols:
    try:
        # Convert to numeric, drop NaNs
        y = pd.to_numeric(df[col], errors='coerce')
        x = df[time_col]
        mask = (~x.isna()) & (~y.isna())
        x = x[mask]
        y = y[mask]

        if len(x) == 0 or len(y) == 0:
            print(f"⚠️ Skipping column '{col}': no valid data to plot.")
            continue

        plt.figure(figsize=(10, 4))
        plt.plot(x, y, label=col)
        plt.xlabel(time_col)
        plt.ylabel(col)
        plt.title(f"{col} vs {time_col}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plot_path = os.path.join(plot_dir, f"{col}.png")
        plt.savefig(plot_path)
        print(f"✅ Saved plot: {plot_path}")
        plt.close()
    except Exception as e:
        print(f"⚠️ Failed to plot {col}: {e}")

print("✅ All available plots saved.")
