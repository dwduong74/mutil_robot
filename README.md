# Không gian làm việc ROS 2 mutil_robot

Kho lưu trữ này chứa một không gian làm việc ROS 2 (ROS 2 workspace), có thể dành cho một dự án đa robot.

## Các gói trong không gian làm việc

- `mutil_bringup`: Gói khởi chạy chính cho hệ thống robot.

## Cách biên dịch (Build)

1.  Di chuyển đến thư mục gốc của không gian làm việc:
    ```bash
    cd mutil_robot
    ```
2.  Biên dịch không gian làm việc bằng `colcon`:
    ```bash
    colcon build
    ```

## Cách chạy

Sau khi biên dịch thành công, hãy thực thi tệp cài đặt để các gói có sẵn trong terminal của bạn:

```bash
source install/setup.bash
```

Sau đó, bạn có thể sử dụng `ros2 launch` hoặc `ros2 run` để khởi chạy các node từ gói `mutil_bringup`.