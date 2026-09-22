#!/bin/bash
set -e

# 1. 載入全域 ROS 2 環境 (Jazzy)
source /opt/ros/jazzy/setup.bash

# 2. 建立 ROS 2 工作區並建立 src 目錄
cd ~
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 3. 建立 Python 套件 (my_py_pkg) 與 C++ 套件 (my_cpp_pkg)
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp

# -------------------------------------------------------------
# 4. 撰寫 Python 節點 (含 Timer 與 Counter)
# -------------------------------------------------------------
cat << 'EOF' > ~/ros2_ws/src/my_py_pkg/my_py_pkg/my_first_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.counter_ = 0
        self.get_logger().info("Hello World")
        self.timer_ = self.create_timer(1.0, self.print_hello)

    def print_hello(self):
        self.get_logger().info(f"Hello {self.counter_}")
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x ~/ros2_ws/src/my_py_pkg/my_py_pkg/my_first_node.py

# 設定 Python setup.py 的 entry_points
cat << 'EOF' > ~/ros2_ws/src/my_py_pkg/setup.py
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_py_pkg.my_first_node:main"
        ],
    },
)
EOF

# -------------------------------------------------------------
# 5. 撰寫 C++ 節點 (含 Timer 與 Counter)
# -------------------------------------------------------------
cat << 'EOF' > ~/ros2_ws/src/my_cpp_pkg/src/my_first_node.cpp
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MyCustomNode : public rclcpp::Node
{
public:
    MyCustomNode() : Node("my_node_name"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello World");
        timer_ = this->create_wall_timer(
            1s, std::bind(&MyCustomNode::print_hello, this));
    }

private:
    void print_hello()
    {
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_++);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
EOF

# 設定 C++ CMakeLists.txt (加入可執行檔與安裝路徑)
cat << 'EOF' > ~/ros2_ws/src/my_cpp_pkg/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
EOF

# -------------------------------------------------------------
# 6. 建置工作區與載入環境變數
# -------------------------------------------------------------
cd ~/ros2_ws
colcon build --symlink-install

# 將工作區環境載入指令寫入 ~/.bashrc (若尚未加入)
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# 載入當前 session 環境變數
source ~/ros2_ws/install/setup.bash

echo "=== 設定與建置完成 ==="
echo "可使用以下指令測試執行 Python 節點："
echo "ros2 run my_py_pkg test_node"
echo "可使用以下指令測試執行 C++ 節點："
echo "ros2 run my_cpp_pkg test_node"
