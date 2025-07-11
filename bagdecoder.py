import sys
import bagpy
from bagpy import bagreader
import pandas as pd
import os

if len(sys.argv) < 2:
    print("Usage: python script.py <bag_file_name_without_extension>")
    sys.exit(1)

bag_name = sys.argv[1]
bag_file = bag_name + ".bag"

# Read bag file
b = bagreader(bag_file)

# Create a directory to store all CSVs
output_dir = bag_name + "_csv"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Extract and save CSV for each topic
for topic in b.topics:
    print(f"Extracting topic: {topic}")
    try:
        csv_path = b.message_by_topic(topic)
        df = pd.read_csv(csv_path)
        # Sanitize topic name for filename
        topic_clean = topic.strip("/").replace("/", "_")
        output_csv = os.path.join(output_dir, f"{topic_clean}.csv")
        df.to_csv(output_csv, index=False)
        print(f"Saved: {output_csv}")
    except Exception as e:
        print(f"Failed to extract topic {topic}: {e}")

print("All available topics have been extracted.")