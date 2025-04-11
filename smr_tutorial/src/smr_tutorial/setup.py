from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smr_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vivit',
    maintainer_email='vivit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'example1 = smr_tutorial.example1:main',
        'example2 = smr_tutorial.example2:main',
        'example3 = smr_tutorial.example3:main',
        'example4 = smr_tutorial.example4:main',
        ],
    },
)
