from setuptools import setup
from glob import glob
import os

package_name                  = 'atmo'
mpc_submodule                 = 'atmo/mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,mpc_submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='imandralis@caltech.edu',
    description='Control package for ATMO, the Aerially Transforming Morphobot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tilt_controller_hardware = atmo.tilt_controller_hardware:main',
            'tilt_controller_sim = atmo.tilt_controller_sim:main',
            'drive_controller_hardware = atmo.drive_controller_hardware:main',
            'drive_controller_sim = atmo.drive_controller_sim:main',
            'mpc_controller_hardware = atmo.mpc_controller_hardware:main',
            'mpc_controller_sim = atmo.mpc_controller_sim:main',
            'load_cell_test = atmo.load_cell_test:main',
            'relay_mocap = atmo.relay_mocap:main'
        ],
    },
)
