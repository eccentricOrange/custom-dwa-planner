from pathlib import Path
from setuptools import find_packages, setup

package_name = 'dwa_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install the launch folder
        (str(Path('share') / package_name / 'launch'),
            [str(p) for p in Path('launch').glob('*.launch.py')]),
            
        # Install the config folder
        (str(Path('share') / package_name / 'config'),
            [str(p) for p in Path('config').glob('*.yaml')]),
            
        # Install the rviz folder
        (str(Path('share') / package_name / 'rviz'),
            [str(p) for p in Path('rviz').glob('*.rviz')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eccentric Orange',
    maintainer_email='eccentric.orange2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwa_planner_node = dwa_planner.dwa_planner_node:main'
        ],
    },
)
