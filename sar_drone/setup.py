from setuptools import find_packages, setup

package_name = 'sar_drone'

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
    maintainer='harmehar',
    maintainer_email='sarru.mehar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    'console_scripts': [
        'flight_controller = sar_drone.flight_controller:main',
        'geotagger = sar_drone.geotagger:main',
        'position_estimator = sar_drone.position_estimator:main',
        'results_logger = sar_drone.results_logger:main',
    ],
},
)
