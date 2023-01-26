from setuptools import setup

package_name = 'ariac_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/' + 'NIST_logo.png']),
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
            'zeid_gui = ariac_gui.zeid_ariac_gui:main'
        ],
    },
)
