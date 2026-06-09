from setuptools import find_packages, setup

package_name = 'flight_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anujjj_k',
    maintainer_email='anujjj_k@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'takeoff_node = flight_control.commander:main',
            'land_node = flight_control.lander:main',
            'mission_node = flight_control.mission:main',
            'harmehar_node = flight_control.harmehar_test:main'
        ],
    },
)
