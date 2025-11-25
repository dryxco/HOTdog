from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/perception']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suyeon Woo',
    maintainer_email='jenifer7933@snu.ac.kr',
    description='Perception package for object detection and localization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_node = perception.nodes.perception_node:main',
        ],
    },
)
