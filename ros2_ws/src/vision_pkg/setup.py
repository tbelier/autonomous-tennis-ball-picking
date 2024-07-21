from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='4.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models/weights_coco128'), glob('models/weights_coco128/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clara',
    maintainer_email='clara.gondot@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_ball    = vision_pkg.detect_ball:main',
            'detect_robot   = vision_pkg.detect_robot:main',
            'choose_closest = vision_pkg.choose_closest:main',
        ],
    },
)
