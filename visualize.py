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

# Try to detect time column
time_col = None
for col in df.columns:
    if 'time' in col.lower():
        time_col = col
        break

if time_col is None:
    print("⚠️ No time column found. Using row index as x-axis.")
    df['index'] = df.index
    time_col = 'index'

# Get numeric columns (excluding time)
numeric_cols = df.select_dtypes(include='number').columns.tolist()
numeric_cols = [col for col in numeric_cols if col != time_col]

if not numeric_cols:
    print("❌ No numeric columns to plot.")
    exit()

# Plot each numeric column
for col in numeric_cols:
    plt.figure(figsize=(10, 4))
    plt.plot(df[time_col], df[col], label=col)
    plt.xlabel(time_col)
    plt.ylabel(col)
    plt.title(f"{col} vs {time_col}")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Save plot
    plot_path = os.path.join(plot_dir, f"{col}.png")
    plt.savefig(plot_path)
    print(f"✅ Saved plot: {plot_path}")

    # Optional: show plot
    # plt.show()

print("✅ All plots saved.")
