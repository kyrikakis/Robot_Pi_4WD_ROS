from setuptools import setup

package_name = 'rainbow_voice'

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
    maintainer='Stefanos Kyrikakis',
    maintainer_email='kirikakis@gmail.com',
    description="Rainbow's voice: text to speech.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speech = rainbow_voice.speech_node:main',
        ],
    },
)
