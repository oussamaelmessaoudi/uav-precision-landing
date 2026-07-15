from setuptools import setup

package_name = 'farasha_landing'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ELMESSAOUDI Oussama',
    maintainer_email='oussama@farasha.systems',
    description='FARASHA Autonomous Precision Landing System',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag_detector          = farasha_landing.apriltag_detector:main',
            'landing_target_publisher   = farasha_landing.landing_target_publisher:main',
            'precision_landing_controller = farasha_landing.precision_landing_controller:main',
            'mavlink_monitor            = farasha_landing.mavlink_monitor:main',
        ],
    },
)
