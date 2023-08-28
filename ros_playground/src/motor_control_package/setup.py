from setuptools import find_packages, setup

package_name = 'motor_control_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/launch',
         ['launch/multi_node_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xl',
    maintainer_email='xutengl@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_node = motor_control_package.decision_node:main',
            'motor_cmd_subscriber = motor_control_package.motor_cmd_subscriber:main',
        ],
    },
)
