import os
from glob import glob
from setuptools import setup

package_name = 'hal_ati_ft_mini45'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resources'), glob(package_name+'/*.xml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob(package_name+'/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Lapolla',
    maintainer_email='marco.lapolla5@gmail.com',
    description='HAL for ATI FT Sensor',
    license='BSD',
    entry_points={
        'console_scripts': [
                'hal_ati_ft_mini45 = hal_ati_ft_mini45.ros2_hal_ati_ft_mini45:main',
        ],
},
)
