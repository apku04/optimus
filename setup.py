from setuptools import find_packages, setup

package_name = 'optimus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['optimus/config.json']),
        ('share/' + package_name + '/launch', ['launch/optimus.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acp',
    maintainer_email='acp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_node = optimus.sensor_node:main',
            'klipper_bridge = optimus.klipper_bridge:main',
            'oled_node = optimus.oled_node:main',
            'hardware_node = optimus.hardware_node:main',
            'voice_node = optimus.voice_node:main',
            'motor_calibration = optimus.motor_calibration_server:main',
        ],
    },
)
