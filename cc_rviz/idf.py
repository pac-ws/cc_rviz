#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class IDFPublisher(Node):
    def __init__(self):
        super().__init__('idf_publisher')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.make_transforms('idf')

        self.system_map_subs = self.create_subscription(
            Float32MultiArray,
            'system_map',
            self.system_map_callback,
            1
        )

        self.system_map_publisher = self.create_publisher(
            PointCloud2,
            '/idf/system',
            1
        )


        self.global_map_publisher = self.create_publisher(
            PointCloud2,
            '/idf/global',
            1
        )

        self.global_map_subs = self.create_subscription(
            Float32MultiArray,
            'global_map',
            self.global_map_callback,
            1
        )

    def system_map_callback(self, msg):
        self.system_map_publisher.publish(self.idf_to_pc(msg, 'idf'))

    def global_map_callback(self, msg):
        self.global_map_publisher.publish(self.idf_to_pc(msg, 'idf'))

    def make_transforms(self, child_frame_id):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = child_frame_id
        
        t.transform.translation.x = float(0)
        t.transform.translation.y = float(0)
        t.transform.translation.z = float(0)
        t.transform.rotation.x = float(0)
        t.transform.rotation.y = float(0)
        t.transform.rotation.z = float(0)
        t.transform.rotation.w = float(1)
        
        self.tf_static_broadcaster.sendTransform(t)

    def idf_to_pc(self, msg, frame_id, scale=1.):
        # rclpy.logging.get_logger('rclpy.node').info('Received IDF matrix')
        # Assuming the matrix is a 1D array, reshape it to 2D
        idf_matrix = np.array(msg.data).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size))

        pc_msg = PointCloud2()

        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = frame_id

        # Set the dimensions
        # pc_msg.width = idf_matrix.shape[1]  # Number of columns
        # pc_msg.height = idf_matrix.shape[0]  # Number of rows
        # Define fields (x, y, z, rgb)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.fields = fields

        # Calculate the point step (size of one point in bytes)
        pc_msg.point_step = 16  # 4 bytes for each of x, y, z, r, g, b

        # Calculate the row step (size of one row in bytes)
        # pc_msg.row_step = pc_msg.point_step * pc_msg.width

        # Define color mapping
        color_min = np.array([255, 255, 255])  # White
        color_max = np.array([162, 40, 47])    # RGB(162, 40, 47)

        # Create point cloud data
        non_neg_indices = np.argwhere(idf_matrix != -1)
        num_points = non_neg_indices.shape[0]
        points = np.zeros((num_points, 4))

        points[:, :2] = np.array(non_neg_indices, dtype=np.float32)*scale
        points[:, 3] = idf_matrix[non_neg_indices[:, 0], non_neg_indices[:, 1]]

        pc_msg.width = len(points)  # Number of columns
        pc_msg.height = 1  # Number of rows

        pc_msg.row_step = pc_msg.point_step * pc_msg.width

        pc_msg.data = np.array(points, dtype=np.float32).tobytes()

        return pc_msg
        # self.publisher.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    idf_publisher = IDFPublisher()
    while rclpy.ok():
        rclpy.spin(idf_publisher)
    idf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
