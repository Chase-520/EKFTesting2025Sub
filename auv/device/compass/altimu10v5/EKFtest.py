import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
from datetime import datetime
import numpy as np
import pandas as pd
from auv.utils.SimEKF import SensorSimulator, DataVisualizer
from auv.device.compass.altimu10v5.sensorfuse_kf import SensorFuse
import cv2
import time

class EKFTester:
    def __init__(self):
        # Create simulator and generate data
        self.simulator = SensorSimulator(
            side_length=5.0,
            imu_rate=100,  # 100 Hz
            dvl_rate=10    # 10 Hz
        )
        self.imu_data, self.dvl_data = self.simulator.generate_square_path()
        
        # Convert to DataFrames and sort by time
        self.imu_df = pd.DataFrame(self.imu_data).sort_values('Time')
        self.dvl_df = pd.DataFrame(self.dvl_data).sort_values('time')
        
        # Create mock DVL object with expected interface
        mock_dvl = type('DVL', (), {
            'vel_rot': [0, 0, 0],  # Initial velocities
            'sub_type': 'onyx',    # Or 'graey' depending on your setup
            'dt': 1.0/10           # DVL time step (10 Hz)
        })()
        
        # Initialize EKF with mock DVL and time step
        self.ekf = SensorFuse(mock_dvl, use_simulated_data=True)
        self.ekf.dt = 1.0/100      # IMU time step (100 Hz)
        
        # Storage for EKF results
        self.ekf_results = []
        
    def simulate_live_processing(self):
        """Process data sequentially to mimic live operation"""
        print("Starting simulated live processing...")
        
        # Get start times
        start_time = min(self.imu_df['Time'].min(), self.dvl_df['time'].min())
        end_time = max(self.imu_df['Time'].max(), self.dvl_df['time'].max())
        
        # Process data in time order
        current_time = start_time
        while current_time <= end_time:
            # Get IMU data at current time
            imu_msg = self.imu_df[self.imu_df['Time'] == current_time]
            if not imu_msg.empty:
                # Create IMU message in correct format
                imu_data = {
                    'linear_acceleration': {
                        'x': imu_msg['linear_acceleration.x'].iloc[0],
                        'y': imu_msg['linear_acceleration.y'].iloc[0],
                        'z': imu_msg['linear_acceleration.z'].iloc[0]
                    }
                }
                # Update EKF with IMU data
                self.ekf.imu_callback(imu_data)
            
            # Get DVL data at current time
            dvl_msg = self.dvl_df[self.dvl_df['time'] == current_time]
            if not dvl_msg.empty:
                # Create DVL data in correct format (matching dvl.py format)
                dvl_data = {
                    'vel_rot': [
                        dvl_msg['vx'].iloc[0],
                        dvl_msg['vy'].iloc[0],
                        dvl_msg['vz'].iloc[0]
                    ]
                }
                # Update EKF with DVL data
                self.ekf.DVL.vel_rot = dvl_data['vel_rot']
                self.ekf.update_filter()
            
            # Store EKF results
            self.ekf_results.append({
                'time': current_time,
                'x': self.ekf.position[0],
                'y': self.ekf.position[1],
                'z': self.ekf.position[2],
                'vx': self.ekf.ekf.x[0],
                'vy': self.ekf.ekf.x[1],
                'vz': self.ekf.ekf.x[2]
            })
            
            # Increment time
            current_time += self.ekf.dt
        
        # Convert results to DataFrame
        self.ekf_df = pd.DataFrame(self.ekf_results)
        print("Simulated live processing completed.")

    def plot(self):
        import matplotlib.pyplot as plt
        
        ekf_results = self.ekf_results
        times = [entry['time'] for entry in ekf_results]
        
        x = [entry['x'] for entry in ekf_results]
        y = [entry['y'] for entry in ekf_results]
        z = [entry['z'] for entry in ekf_results]
        
        vx = [entry['vx'] for entry in ekf_results]
        vy = [entry['vy'] for entry in ekf_results]
        vz = [entry['vz'] for entry in ekf_results]
        
        fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # Position
        axs[0].plot(times, x, label='x')
        axs[0].plot(times, y, label='y')
        axs[0].plot(times, z, label='z')
        axs[0].set_ylabel('Position')
        axs[0].legend()
        axs[0].grid(True)

        # Velocity
        axs[1].plot(times, vx, label='vx')
        axs[1].plot(times, vy, label='vy')
        axs[1].plot(times, vz, label='vz')
        axs[1].set_ylabel('Velocity')
        axs[1].set_xlabel('Time')
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_comparison(self, output_dir):
        """Plot comparison of raw and EKF-processed data and save to file"""
        visualizer = DataVisualizer()
        
        # Plot raw simulated data
        visualizer.load_data(self.imu_df, self.dvl_df, None)
        raw_output_file = os.path.join(output_dir, 'raw_simulated_data.png')
        visualizer.plot_all(show_ekf=False, output_file=raw_output_file)
        
        # Plot EKF-processed data
        visualizer.load_data(self.imu_df, self.dvl_df, self.ekf_df)
        ekf_output_file = os.path.join(output_dir, 'ekf_processed_data.png')
        visualizer.plot_all(show_ekf=True, output_file=ekf_output_file)
        
        print(f"Raw simulated data plot saved to {raw_output_file}")
        print(f"EKF processed data plot saved to {ekf_output_file}")

if __name__ == "__main__":
    # Create a directory for this run
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('EKFComp', timestamp)
    os.makedirs(output_dir, exist_ok=True)
    
    tester = EKFTester()
    tester.simulate_live_processing()
    
    # directly show the result of ekf
    tester.plot()
    # Plot comparison of raw and EKF-processed data and save to file
    # tester.plot_comparison(output_dir=output_dir)
