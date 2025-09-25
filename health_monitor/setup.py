from setuptools import find_packages, setup

package_name = 'health_monitor'

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
    maintainer='avishai-shustak',
    maintainer_email='avishai-shustak@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "health_analyzer_node = health_monitor.health_analyzer_node:main",
            "alert_manager_node = health_monitor.alert_manager_node:main",
            "can_ros2_status_node = health_monitor.can_ros2_status_node:main"
        ],
    },
)
