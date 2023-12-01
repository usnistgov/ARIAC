from setuptools import setup

package_name = 'ariac_gui'
PART_TYPES=["sensor", "pump", "regulator", "battery"]
PART_COLORS=['green', 'red', 'purple','blue','orange']
file_names = ["plus", 'NIST_logo']+[color+pType for color in PART_COLORS for pType in PART_TYPES]
setup(
    name=package_name,
    version='0.0.0',
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
    maintainer_email='jothomas0615@gmail.com',
    description='For testing ros 2',
    license='ARIAC',
    entry_points={
        'console_scripts': [
            'gui = ariac_gui.gui_node:main',
            'zeid_gui = ariac_gui.zeid_ariac_gui:main',
            'new_gui = ariac_gui.new_gui_node:main'
        ],
    },
)
