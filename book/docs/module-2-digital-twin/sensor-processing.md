---
sidebar_position: 10
---

# Sensor Data Processing in Simulation

Processing sensor data in simulation is a critical aspect of robotic systems. While simulated sensors provide idealized data, processing this data effectively requires understanding the characteristics and limitations of both the sensors and the simulated environment.

## Overview of Sensor Data Processing

Sensor data processing in simulation involves:
- Receiving sensor messages from simulated sensors
- Filtering and interpreting the data
- Using the processed data for perception, navigation, or control
- Validating results against expected simulation behavior

## Processing Camera Data

### Image Subscription and Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Publish or use processed data
        self.process_edges(edges)
        
    def process_edges(self, edges):
        # Find contours in the edge image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours (e.g., find largest contour)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            # Calculate bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            self.get_logger().info(f'Object detected at: ({x}, {y}) with size ({w}, {h})')
```

### Depth Image Processing

```python
from sensor_msgs.msg import Image
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            10)
        self.bridge = CvBridge()
        
    def depth_callback(self, msg):
        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
        # Find depth at center of image
        height, width = depth_image.shape
        center_depth = depth_image[height//2, width//2]
        
        if not np.isnan(center_depth) and center_depth > 0:
            self.get_logger().info(f'Distance to object at center: {center_depth:.2f}m')
            
        # Calculate depth statistics
        valid_depths = depth_image[np.isfinite(depth_image) & (depth_image > 0)]
        if valid_depths.size > 0:
            avg_depth = np.mean(valid_depths)
            min_depth = np.min(valid_depths)
            max_depth = np.max(valid_depths)
            
            self.get_logger().info(
                f'Depth stats - Avg: {avg_depth:.2f}m, '
                f'Min: {min_depth:.2f}m, Max: {max_depth:.2f}m'
            )
```

## Processing LiDAR Data

### LaserScan Message Processing

```python
from sensor_msgs.msg import LaserScan
import numpy as np

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
    def scan_callback(self, msg):
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)
        
        # Filter out invalid ranges (inf, nan)
        valid_indices = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))[valid_indices]
        
        if len(valid_ranges) > 0:
            # Find closest obstacle
            min_distance_idx = np.argmin(valid_ranges)
            min_distance = valid_ranges[min_distance_idx]
            min_angle = valid_angles[min_angle_idx]
            
            self.get_logger().info(
                f'Closest obstacle: {min_distance:.2f}m at angle {min_angle:.2f}rad'
            )
            
            # Detect obstacles in front of robot (e.g., within 30 degrees)
            front_mask = (valid_angles >= -np.pi/6) & (valid_angles <= np.pi/6)
            front_ranges = valid_ranges[front_mask]
            
            if len(front_ranges) > 0:
                min_front_distance = np.min(front_ranges)
                if min_front_distance < 1.0:  # Less than 1m
                    self.get_logger().warn('Obstacle detected in front! Distance: {:.2f}m'.format(min_front_distance))
```

### Point Cloud Processing

```python
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10)
        
    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to list of points
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert to numpy array
        points_array = np.array(list(points))
        
        if len(points_array) > 0:
            # Calculate bounding box
            min_vals = np.min(points_array, axis=0)
            max_vals = np.max(points_array, axis=0)
            center = (min_vals + max_vals) / 2
            
            self.get_logger().info(
                f'Point cloud bounds: X({min_vals[0]:.2f}, {max_vals[0]:.2f}), '
                f'Y({min_vals[1]:.2f}, {max_vals[1]:.2f}), '
                f'Z({min_vals[2]:.2f}, {max_vals[2]:.2f})'
            )
            
            # Cluster detection (simplified approach)
            self.detect_clusters(points_array)
    
    def detect_clusters(self, points):
        # Simple clustering based on z-height
        ground_level = np.mean(points[:, 2]) - np.std(points[:, 2])
        ground_points = points[points[:, 2] < ground_level + 0.1]
        obstacle_points = points[points[:, 2] >= ground_level + 0.1]
        
        self.get_logger().info(
            f'Ground points: {len(ground_points)}, '
            f'Obstacle points: {len(obstacle_points)}'
        )
```

## Processing IMU Data

```python
from sensor_msgs.msg import Imu
import numpy as np

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
    def imu_callback(self, msg):
        # Extract orientation (quaternion)
        orientation = msg.orientation
        # Convert quaternion to Euler angles (simplified)
        euler = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Extract angular velocity
        angular_velocity = msg.angular_velocity
        
        # Extract linear acceleration
        linear_accel = msg.linear_acceleration
        
        self.get_logger().info(
            f'Orientation: Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}\n'
            f'Angular Vel: X={angular_velocity.x:.2f}, Y={angular_velocity.y:.2f}, Z={angular_velocity.z:.2f}\n'
            f'Linear Acc: X={linear_accel.x:.2f}, Y={linear_accel.y:.2f}, Z={linear_accel.z:.2f}'
        )
        
    def quaternion_to_euler(self, x, y, z, w):
        # Simplified conversion (not accounting for all edge cases)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return [roll, pitch, yaw]
```

## Sensor Fusion

### Combining Multiple Sensors

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseStamped
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Publisher
        self.obstacle_pub = self.create_publisher(PoseStamped, '/obstacle_pose', 10)
        
        # State variables
        self.latest_imu = None
        self.latest_scan = None
        
    def scan_callback(self, msg):
        self.latest_scan = msg
        self.process_fusion()
        
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.process_fusion()
        
    def process_fusion(self):
        if self.latest_scan is None or self.latest_imu is None:
            return
            
        # Find closest obstacle from LiDAR
        ranges = np.array(self.latest_scan.ranges)
        valid_ranges = np.isfinite(ranges) & (ranges > 0)
        
        if np.any(valid_ranges):
            min_idx = np.argmin(ranges[valid_ranges])
            min_range = ranges[valid_ranges][min_idx]
            
            # Calculate angle of closest obstacle
            angle_increment = self.latest_scan.angle_increment
            obstacle_angle = self.latest_scan.angle_min + min_idx * angle_increment
            
            # Use IMU orientation to transform to global frame
            imu_orientation = self.latest_imu.orientation
            euler = self.quaternion_to_euler(
                imu_orientation.x, imu_orientation.y, 
                imu_orientation.z, imu_orientation.w
            )
            
            # Calculate global position of obstacle
            # (Simplified - in practice, you'd use full TF transforms)
            global_angle = obstacle_angle + euler[2]  # Add yaw from IMU
            obstacle_x = min_range * np.cos(global_angle)
            obstacle_y = min_range * np.sin(global_angle)
            
            # Publish obstacle position
            obstacle_pose = PoseStamped()
            obstacle_pose.header.stamp = self.get_clock().now().to_msg()
            obstacle_pose.header.frame_id = 'map'
            obstacle_pose.pose.position.x = obstacle_x
            obstacle_pose.pose.position.y = obstacle_y
            obstacle_pose.pose.position.z = 0.0
            
            self.obstacle_pub.publish(obstacle_pose)
```

## Filtering and Noise Handling

### Basic Filtering for Sensor Data

```python
class SensorFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []
        
    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        
        return sum(self.values) / len(self.values)  # Simple moving average
    
    def get_filtered_value(self):
        if not self.values:
            return None
        return sum(self.values) / len(self.values)

# Example usage in a sensor processor
class FilteredSensorProcessor(Node):
    def __init__(self):
        super().__init__('filtered_sensor_processor')
        self.distance_filter = SensorFilter(window_size=5)
        
    def scan_callback(self, msg):
        # Get minimum distance from scan
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
        
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            filtered_distance = self.distance_filter.update(min_distance)
            
            if filtered_distance is not None:
                self.get_logger().info(f'Raw min distance: {min_distance:.2f}, Filtered: {filtered_distance:.2f}')
```

## Performance Considerations

### Efficient Data Processing

1. **Use appropriate data types**: Use numpy arrays for numerical computations
2. **Batch processing**: Process multiple data points together when possible
3. **Threading**: Use separate threads for computationally intensive tasks
4. **Message filtering**: Process only necessary messages at required rates

### Memory Management

```python
# Limit stored data to prevent memory issues
class RingBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = []
        
    def add(self, item):
        self.buffer.append(item)
        if len(self.buffer) > self.size:
            self.buffer.pop(0)
            
    def get_all(self):
        return self.buffer.copy()
```

## Validation Techniques

### Comparing Simulated vs. Expected Data

```python
class SensorValidator:
    def __init__(self):
        self.expected_values = {}
        self.tolerance = 0.1  # Tolerance for validation
        
    def validate_camera_data(self, image, expected_objects):
        # Process image to detect objects
        # Compare with expected objects
        detected_objects = self.detect_objects(image)
        
        for expected_obj in expected_objects:
            found = False
            for detected_obj in detected_objects:
                if self.objects_match(expected_obj, detected_obj):
                    found = True
                    break
            
            if not found:
                print(f"Expected object {expected_obj} not found in simulation")
    
    def objects_match(self, obj1, obj2):
        # Implement logic to determine if objects match
        # This would depend on your specific object representation
        pass
```

Sensor data processing in simulation requires understanding both the characteristics of the simulated sensors and the algorithms needed to extract meaningful information from the data. Proper processing enables robots to effectively perceive and interact with their simulated environment.