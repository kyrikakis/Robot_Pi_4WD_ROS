from setuptools import setup

package_name = 'rainbow_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='Stefanos Kyrikakis',
    maintainer_email='kirikakis@gmail.com',
    description="Rainbow's hardware nodes: head, eyes, ToF, velocity watchdog, IP publisher.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'head = rainbow_drivers.head_node:main',
            'eyes = rainbow_drivers.eyes_node:main',
            'tof = rainbow_drivers.tof_node:main',
            'cmd_vel_watchdog = rainbow_drivers.cmd_vel_watchdog:main',
            'ip_publisher = rainbow_drivers.ip_publisher:main',
            'plan_viz = rainbow_drivers.plan_viz:main',
        ],
    },
)
