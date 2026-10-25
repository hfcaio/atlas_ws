from glob import glob

from setuptools import find_packages, setup

package_name = 'artex27'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/missions',
            glob('missions/*.waypoints') + glob('missions/*.plan')),
        ('share/' + package_name + '/missions/kml', glob('missions/kml/*.kml')),
        ('share/' + package_name + '/params',
            glob('params/*.param') + glob('params/*.md')),
        ('share/' + package_name + '/tools',
            glob('tools/*.py') + glob('tools/*.sh') + glob('tools/*.txt')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Missao ARTEX 26: subida a 2000 m com gradiente maximo de 10 %.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'artex_mission = artex27.nodes.artex_mission_node:main',
            'artex_check = artex27.nodes.artex_check_node:main',
        ],
    },
)
