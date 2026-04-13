import os
from glob import glob

from setuptools import setup

package_name = 'rqt_quadrotor_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        (os.path.join('share', package_name, 'resource'),
            glob('resource/*.ui') + glob('resource/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dinesh Thakur',
    maintainer_email='ablasdel@gmail.com',
    description='Provides a GUI plugin for MAV Manager.',
    license='Penn Software License',
    entry_points={
        'console_scripts': [
            'rqt_quadrotor_safety = rqt_quadrotor_safety.main:main',
        ],
    },
)
