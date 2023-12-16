from setuptools import setup
from glob import glob
import os

package_name = 'epsilon_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tim',
    maintainer_email='tim.dodge64@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_control = epsilon_control.epsilon_point_controller:main',
            'state_plotter = epsilon_control.state_plotting:main',
        ],
    },
)
