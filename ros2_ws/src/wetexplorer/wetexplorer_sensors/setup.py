from setuptools import find_packages, setup

package_name = 'wetexplorer_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_processing_node = wetexplorer_sensors.imu_processing_node:main',
            'odometry_real = wetexplorer_sensors.odometry_real:main',
            'odometry_sim = wetexplorer_sensors.odometry_sim:main',
        ],
},
)
