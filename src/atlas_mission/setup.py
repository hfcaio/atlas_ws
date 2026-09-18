from glob import glob
from setuptools import find_packages, setup

package_name = 'atlas_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'missao1 = atlas_mission.nodes.mission1_node:main',
            'missao2 = atlas_mission.nodes.mission2_node:main',
            'missao3 = atlas_mission.nodes.mission3_node:main',
            'fake_target = atlas_mission.nodes.fake_target:main',
        ],
    },
)
