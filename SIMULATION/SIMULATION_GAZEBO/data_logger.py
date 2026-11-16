#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib
matplotlib.use('TkAgg') # Force external window
import matplotlib.pyplot as plt
import os

class LivePlotter(Node):
    def __init__(self):
        super().__init__("live_plotter")
        self.sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        
        self.times = []
        self.torques = []
        self.start_time = None
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line, = self.ax.plot([], [], 'r-', label="Max Absolute Torque (N.m)")
        self.ax.set_title("Live Joint Torque Data (Gazebo)")
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Torque [N.m]")
        self.ax.legend()
        self.ax.grid(True)
        
        self.get_logger().info("Live Plotter started. Look for the popup window!")

    def joint_state_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
            
        t = now - self.start_time
        
        if len(msg.effort) >= 12:
            max_tau = max([abs(e) for e in msg.effort[:12]])
            
            self.times.append(t)
            self.torques.append(max_tau)
            
            if len(self.times) > 500:
                self.times.pop(0)
                self.torques.pop(0)
            
            self.line.set_data(self.times, self.torques)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    plotter = LivePlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        out_dir = os.path.expanduser("~/robotdog_ws/sim_out")
        os.makedirs(out_dir, exist_ok=True)
        plot_path = os.path.join(out_dir, "final_torque_graph.png")
        plotter.fig.savefig(plot_path, dpi=150)
        print(f"\nFinal graph saved to {plot_path}")
        
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
