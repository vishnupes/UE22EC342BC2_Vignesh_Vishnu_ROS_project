from setuptools import setup
import os
from glob import glob

package_name = 'mars_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Use absolute paths for the glob pattern
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(os.path.dirname(__file__), 'launch', '*.py'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join(os.path.dirname(__file__), 'urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join(os.path.dirname(__file__), 'worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Mars rover simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'move_rover = mars_rover.move_rover:main',
        'teleop_rover = mars_rover.teleop_rover:main',  # Add this line
    ],
},
)