from glob import glob

from setuptools import setup

package_name = 'rainbow_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='Stefanos Kyrikakis',
    maintainer_email='kirikakis@gmail.com',
    description="Rainbow's phone remote: a web page served from the robot.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web = rainbow_web.web_node:main',
        ],
    },
)
