from setuptools import find_packages, setup

package_name = 'rover_pilot'

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
    maintainer='bruce',
    maintainer_email='diveshkumar_23ee078@dtu.ac.in',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node_test = rover_pilot.drive_node_test:main',
            'drive_node = rover_pilot.drive_node:main',
            'auto_node = rover_pilot.auto_node:main',
        ],
    },
)
