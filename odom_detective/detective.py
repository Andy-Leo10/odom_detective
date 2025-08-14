#!/usr/bin/env python3

'''
This code subscribes to odometry data and plot them in real-time using matplotlib
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import numpy as np

class Detective(Node):
    def __init__(self):
        super().__init__('detective')
        
        self.declare_parameter('real_life', False)
        self.declare_parameter('odom_topics', ['/odom_matcher', '/odometry/filtered'])
        self.declare_parameter('world_name', 'demo_world')
        
        self.real_life = self.get_parameter('real_life').get_parameter_value().bool_value
        self.odom_topics = self.get_parameter('odom_topics').get_parameter_value().string_array_value
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        
        self.odom_data = {topic: {'x': [], 'y': [], 'start_x': None, 'start_y': None} for topic in self.odom_topics}
        self.ground_truth = {'x': [], 'y': [], 'start_x': None, 'start_y': None}
        self.errors = {topic: [] for topic in self.odom_topics}
                
        for topic in self.odom_topics:
            self.create_subscription(Odometry, topic, self.odom_callback(topic), 10)
            
        if not self.real_life:
            ground_truth_topic = f'/world/{self.world_name}/dynamic_pose/info'
            self.model_states_subscriber = self.create_subscription(
                TFMessage,
                ground_truth_topic,
                self.model_states_callback,
                10
            )
            
        # Initialize the plot
        plt.ion()
        if not self.real_life:
            self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
            self.ax2.set_xlabel('Time')
            self.ax2.set_ylabel('Error')
            self.ax2.set_title('Odometry Error')
        else:
            self.fig, self.ax1 = plt.subplots(figsize=(6, 5))
            
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Odometry History')
        
        plt.show()
        
        self.create_timer(0.1, self.update_plot)
        
        if not self.real_life:
            self.create_timer(0.1, self.calculate_errors)
            
    def odom_callback(self, topic):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if self.odom_data[topic]['start_x'] is None:
                self.odom_data[topic]['start_x'] = x
                self.odom_data[topic]['start_y'] = y
            self.odom_data[topic]['x'].append(x - self.odom_data[topic]['start_x'])
            self.odom_data[topic]['y'].append(y - self.odom_data[topic]['start_y'])
        return callback
    
    def model_states_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'my_robot':
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                if self.ground_truth['start_x'] is None:
                    self.ground_truth['start_x'] = x
                    self.ground_truth['start_y'] = y
                self.ground_truth['x'].append(x - self.ground_truth['start_x'])
                self.ground_truth['y'].append(y - self.ground_truth['start_y'])
                
    def calculate_errors(self):
        if self.ground_truth['x']:
            for topic in self.odom_topics:
                if self.odom_data[topic]['x']:
                    # Ensure both arrays have the same length
                    min_length = min(len(self.odom_data[topic]['x']), len(self.ground_truth['x']))
                    odom_x = np.array(self.odom_data[topic]['x'][:min_length])
                    odom_y = np.array(self.odom_data[topic]['y'][:min_length])
                    ground_truth_x = np.array(self.ground_truth['x'][:min_length])
                    ground_truth_y = np.array(self.ground_truth['y'][:min_length])
                    
                    # Calculate error
                    error = np.sqrt((odom_x - ground_truth_x)**2 + (odom_y - ground_truth_y)**2)
                    self.errors[topic].append(np.mean(error))
    
    def update_plot(self):
        self.ax1.clear()
        
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Odometry History')
        
        for topic in self.odom_topics:
            if self.odom_data[topic]['x']:
                self.ax1.plot(self.odom_data[topic]['x'], self.odom_data[topic]['y'], label=topic)
                
        if not self.real_life and self.ground_truth['x']:
            self.ax1.plot(self.ground_truth['x'], self.ground_truth['y'], label='Ground Truth')
            
            self.ax2.clear()
            self.ax2.set_xlabel('Time')
            self.ax2.set_ylabel('Error')
            self.ax2.set_title('Odometry Error')
            for topic in self.odom_topics:
                if self.errors[topic]:
                    self.ax2.plot(range(len(self.errors[topic])), self.errors[topic], label=f'{topic}')
            self.ax2.legend()
        
        self.ax1.legend()
        
        self.ax1.set_aspect('equal', adjustable='box')
        
        # set axis limits to ensure a minimun axis length of 2.0
        minimun_axis_length = 5.0
        x_min, x_max = self.ax1.get_xlim()
        y_min, y_max = self.ax1.get_ylim()
        
        x_range = x_max - x_min
        y_range = y_max - y_min
        
        if x_range < minimun_axis_length:
            x_center = (x_max + x_min) / 2
            x_min = x_center - (minimun_axis_length / 2)
            x_max = x_center + (minimun_axis_length / 2)
            
        if y_range < minimun_axis_length:
            y_center = (y_max + y_min) / 2
            y_min = y_center - (minimun_axis_length / 2)
            y_max = y_center + (minimun_axis_length / 2)
            
        self.ax1.set_xlim(x_min, x_max)
        self.ax1.set_ylim(y_min, y_max)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
def main(args=None):
    rclpy.init(args=args)
    detective = Detective()
    try:
        rclpy.spin(detective)
    except KeyboardInterrupt:
        pass
    finally:
        detective.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()
        
if __name__ == '__main__':
    main()