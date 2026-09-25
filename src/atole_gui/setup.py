from glob import glob

from setuptools import find_packages, setup

package_name = 'atole_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michell IV',
    maintainer_email='michellvsigno21@gmail.com',
    description='Web GUI: servidor, puente de datos y página (rosbridge + Lichtblick).',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'gui_bridge = atole_gui.gui_bridge:main',
            'gui_server = atole_gui.gui_server:main',
        ],
    },
)
