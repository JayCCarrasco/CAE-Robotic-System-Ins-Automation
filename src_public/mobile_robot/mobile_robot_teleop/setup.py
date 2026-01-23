from setuptools import find_packages, setup

package_name = 'mobile_robot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mobile_robot_teleop.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_joy.yaml']),
        ('share/' + package_name + '/launch', ['launch/mobile_robot_keyboard.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_keyboard.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juancarlos',
    maintainer_email='JayCCarrasco@users.noreply.github.com',
    description='Mobile robot teleoperation control',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
