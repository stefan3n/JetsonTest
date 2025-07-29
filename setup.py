from setuptools import setup

package_name = 'arduino_serial_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 package for Arduino serial communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = arduino_serial_comm.serial_node:main',
        ],
    },
)
