from glob import glob

from setuptools import find_packages, setup

package_name = 'atole_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michell IV',
    maintainer_email='michellvsigno21@gmail.com',
    description='Percepción: detector, pose de pods y fusión de nubes.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'detector_node = atole_perception.detector_node:main',
            'pod_pose_node = atole_perception.pod_pose_node:main',
            'cloud_fusion_node = atole_perception.cloud_fusion_node:main'
        ],
    },
)
