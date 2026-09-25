from glob import glob

from setuptools import find_packages, setup

package_name = 'atole_sim'

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
    description='Modo SIM: reproduce datasets como si fueran las cámaras.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'dataset_player = atole_sim.dataset_player:main'
        ],
    },
)
