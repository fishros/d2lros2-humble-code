from setuptools import setup

package_name = 'example_action_rclpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='fishros@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_robot_02 = example_action_rclpy.action_robot_02:main',
            'action_control_02 = example_action_rclpy.action_control_02:main'
        ],
    },
)
