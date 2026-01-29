from setuptools import find_packages, setup

package_name = 'ariac_app'

setup(
    name=package_name,
    version='2025.3.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Albrecht',
    maintainer_email='justin.albrecht@nist.gov',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'app = ariac_app.app:main',
          'competition_app = ariac_app.competition_app:main'
        ],
    },
)
