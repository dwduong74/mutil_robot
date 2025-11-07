import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# import tf_transformations # Cần cài đặt: pip install tf-transformations

class GlobalMapPublisher(Node):
    def __init__(self):
        super().__init__('global_map_publisher_node')

        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Lấy thời gian hiện tại một lần
        now = self.get_clock().now().to_msg()
        
        # --- Transform 1: global_map -> TB3_1/map ---
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'global_map'  # Frame cha
        t1.child_frame_id = 'TB3_1/map'     # Frame con 1

        # Giả sử vị trí ban đầu trùng nhau (0, 0, 0)
        # Bạn có thể thay đổi các giá trị này nếu map của TB3_1
        # có vị trí offset so với global_map
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0

        # Giả sử không xoay (Quaternion: 0, 0, 0, 1)
        # q1 = tf_transformations.quaternion_from_euler(0, 0, 0) # (roll, pitch, yaw)
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        # --- Transform 2: global_map -> TB3_2/map ---
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'global_map'  # Frame cha
        t2.child_frame_id = 'TB3_2/map'     # Frame con 2

        # Giả sử vị trí ban đầu của TB3_2/map cũng trùng (0, 0, 0)
        # Bạn có thể thay đổi vị trí này, ví dụ:
        # t2.transform.translation.x = 5.0 # (Cách 5m theo trục X)
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0

        # Giả sử không xoay (Quaternion: 0, 0, 0, 1)
        # q2 = tf_transformations.quaternion_from_euler(0, 0, 0) # (roll, pitch, yaw)
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        # --- Publish cả hai transform ---
        # StaticTransformBroadcaster có thể nhận một list các transforms
        self.static_broadcaster.sendTransform([t1, t2])
        
        self.get_logger().info('Published static TFs: global_map -> TB3_1/map AND global_map -> TB3_2/map')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapPublisher()

    # Node đã publish transform trong __init__
    # Chúng ta cần giữ cho node chạy (spin) để các node khác
    # có thể nhận được transform tĩnh từ topic /tf_static.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Dọn dẹp khi tắt node (ví dụ: bằng Ctrl+C)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()