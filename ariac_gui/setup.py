from setuptools import setup

package_name = 'ariac_gui'
PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']
file_names = ['plus','assembly_station','agv','tray','light_icon','dark_icon']+[color+pType for color in PART_COLORS for pType in PART_TYPES]+[f'id_0{i}' for i in range(10)]
setup(
    name=package_name,
    version='2024.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/' + f'{file_name}.png' for file_name in file_names]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jfernandez',
    maintainer_email='joseph.fernandez@nist.gov',
    description='For testing ros 2',
    license='ARIAC',
    entry_points={
        'console_scripts': [
            'trial_generator = ariac_gui.trial_generator_node:main',
        ],
    },
)
