#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from math import pi, cos, sin
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry  # Odometry 메시지를 사용하기 위해 추가
import csv

class TrackVisualizer(Node):
    def __init__(self):
        super().__init__('mobile_system_control_vis')
        
        self.pub_track = self.create_publisher(Marker, "track", 10)
        self.pub_vehicle = self.create_publisher(Marker, "vehicle", 10)
        
        # Odometry 토픽을 구독하도록 변경
        self.sub_pose = self.create_subscription(
            Odometry, "pose", self.posecb, 10
        )
        
        self.declare_parameter('csv_dir', '')
        self.csv_dir = self.get_parameter('csv_dir').get_parameter_value().string_value
        
        self.vehicle = Marker()
        self.vehicle.type = self.vehicle.ARROW
        self.vehicle.header.frame_id = "map"
        self.vehicle.color.a = 1.0
        self.vehicle.color.r = 1.0
        self.vehicle.color.g = 0.0
        self.vehicle.color.b = 0.0
        self.vehicle.scale.x = 4.0
        self.vehicle.scale.y = 1.0
        self.vehicle.scale.z = 2.0

        self.track = Marker()
        self.track.type = self.track.SPHERE_LIST
        self.track.header.frame_id = "map"
        self.track.id = 0
        self.track.color.a = 0.8
        self.track.color.r = 0.0
        self.track.color.g = 1.0
        self.track.color.b = 0.0
        self.track.scale.x = 0.3
        self.track.scale.y = 0.3
        self.track.scale.z = 0.3
        
        self.load_track_points()
        
        self.timer = self.create_timer(0.1, self.publish_markers)

    def load_track_points(self):
        self.track.points = []
        try:
            with open(self.csv_dir, "r") as f:
                rdr = csv.reader(f)
                for line in rdr:
                    if len(line) >= 2:
                        p = Point()
                        p.x = float(line[0])
                        p.y = float(line[1])
                        # z 값은 3번째 열에 있으면 사용하고, 없으면 0.0으로 설정
                        p.z = float(line[2]) if len(line) > 2 else 0.0
                        self.track.points.append(p)
            self.get_logger().info(f'Track loaded successfully from {self.csv_dir}')
        except Exception as e:
            self.get_logger().error(f'Failed to load track file: {e}')

    def posecb(self, msg: Odometry): # 메시지 타입을 Odometry로 변경
        # Odometry 메시지에서 직접 위치와 방향 정보를 사용합니다.
        self.vehicle.pose = msg.pose.pose

    def publish_markers(self):
        # 발행 전에 항상 타임스탬프를 현재 시간으로 업데이트
        self.track.header.stamp = self.get_clock().now().to_msg()
        self.vehicle.header.stamp = self.get_clock().now().to_msg()
        
        self.pub_vehicle.publish(self.vehicle)
        self.pub_track.publish(self.track)

def main(args=None):
    rclpy.init(args=args)
    vis_node = TrackVisualizer()
    try:
        rclpy.spin(vis_node)
    except KeyboardInterrupt:
        pass
    finally:
        vis_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()