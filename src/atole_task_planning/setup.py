from glob import glob

from setuptools import find_packages, setup

package_name = 'atole_task_planning'

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
    description='Selección del pod a cosechar.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'pod_selector = atole_task_planning.pod_selector:main'
        ],
    },
)
