# mutil_robot
⚙️ 1. Khởi tạo robot trong môi trường mô phỏng
Lệnh:
ros2 launch mutil_bringup Spawn_launch.py

Mô tả:

File này khởi tạo các robot TurtleBot3 trong Gazebo hoặc RViz (tuỳ config).

Mỗi robot được gán namespace riêng (TB3_1, TB3_2, …).
Chỉnh number_robot trong file để thêm robot.

🧩 2. Chạy SLAM hoặc Navigation cho từng robot
Lệnh robot 1:
ros2 launch mutil_bringup nav2_slam.launch.py namespace:="TB3_1" use_sim_time:=true

Lệnh robot 2:
ros2 launch mutil_bringup nav2_slam.launch.py namespace:="TB3_2" use_sim_time:=true

Mô tả:

Mỗi lệnh chạy SLAM hoặc Navigation2 trong không gian tên (namespace) riêng. Ô mà chả hiểu sao em gửi goal_pose theo namespace r nhma nó kh đi :v

Tham số:

namespace: định danh robot trong hệ thống đa robot.

use_sim_time:=true: đồng bộ thời gian mô phỏng.

🗺️ 3. Hợp nhất bản đồ toàn cục (Global Map Merge)
Lệnh:
python3 src/mutil_bringup/mutil_bringup/global_map_pub.py

Mô tả:

Script Python để kết hợp bản đồ (map) từ nhiều robot thành một bản đồ toàn cục. Cái này mới nhờ chat GPT viết, em chưa đóng gói, nên thầy sửa lại đúng đường dẫn nhé, đại loại là ghép Tf hai robot .

Thường chạy sau khi các robot đã có bản đồ riêng.

Kết quả: một topic bản đồ tổng hợp được publish (ví dụ /global_map).

📍 4. Định vị toàn cục với AMCL
Lệnh:
ros2 launch mutil_bringup amcl_pose.py

Mô tả:

Khởi chạy node xử lý định vị bằng AMCL (Adaptive Monte Carlo Localization) cho các robot. Cái này em chưa cài tham số, mối đổi thì phải vô file param để sửa.

Cập nhật pose ước lượng của từng robot trong map toàn cục.

Có thể dùng để theo dõi hoặc hiệu chỉnh vị trí ban đầu của robot sau khi hợp nhất bản đồ.

🔄 5. Tái khởi tạo phân phối hạt AMCL (Global Localization Reset)
Lệnh:
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}"

Mô tả:

Gọi service của AMCL để phân bố lại toàn bộ các hạt (particles) trên bản đồ.

Dùng khi robot bị “mất định vị” (lost localization) hoặc khi muốn đặt lại toàn bộ trạng thái ban đầu của AMCL.
