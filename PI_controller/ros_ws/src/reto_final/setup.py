from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reto_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jchr2003',
    maintainer_email='jchr2003@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_generator = reto_final.signal_generator:main',
            'set_point = reto_final.set_point:main',
            'csv_recorder = reto_final.csv_recorder:main'
        ],
    },
)
