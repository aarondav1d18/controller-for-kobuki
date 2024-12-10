from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kobuki_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
	(os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaron',
    maintainer_email='aarond2005@icloud.com',
    description='Make the kobuki move using a controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kobuki_controller = kobuki_controller.kobuki_controller:main'
        ],
    },
)
