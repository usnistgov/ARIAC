from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ariac_setup'

setup(
    name=package_name,
    version='2025.3.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joseph Fernandez',
    maintainer_email='joseph.fernandez@nist.gov',
    description='Python nodes for setting up the environment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'startup = ariac_setup.startup:main',
          'ready = ariac_setup.ready:main',
          'score_logger = ariac_setup.score_logger:main'
        ],
    },
)
