from setuptools import find_packages
from setuptools import setup

package_name = 'launch_ros_experimental'

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
    maintainer='Christoph Hellmann Santos',
    maintainer_email='christoph.hellmann.santos@ipa.fraunhofer.de',
    description='Refactored ros2 composition system for launch',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': [
            'launch_ros_experimental = launch_ros_experimental',
        ],
    },
)
