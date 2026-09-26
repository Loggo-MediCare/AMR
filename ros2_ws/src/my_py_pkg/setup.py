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
            "test_node = my_py_pkg.my_first_node:main",
            'amr_status_publisher = my_py_pkg.amr_status_publisher:main',
            'amr_status_subscriber = my_py_pkg.amr_status_subscriber:main',
            'calendar_navigation_agent = my_py_pkg.calendar_navigation_agent:main',
            'mock_navigate_to_pose_server = my_py_pkg.mock_navigate_to_pose_server:main',
            'mock_odom_publisher = my_py_pkg.mock_odom_publisher:main',
            'encoder_serial_bridge = my_py_pkg.encoder_serial_bridge:main',
            'wheel_odometry_node = my_py_pkg.wheel_odometry_node:main',
            'simulated_encoder_ticks = my_py_pkg.simulated_encoder_ticks:main',
            'offline_encoder_odometry_test = my_py_pkg.offline_encoder_odometry_test:main',
            'simulated_imu_publisher = my_py_pkg.simulated_imu_publisher:main',
            'imu_message_validator = my_py_pkg.imu_message_validator:main',
],
    },
)
