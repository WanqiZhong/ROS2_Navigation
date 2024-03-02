from setuptools import find_packages, setup

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serial_bridge.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='future',
    maintainer_email='7175380@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'serial_read = serial_bridge.serial_read:main',
                'serial_write = serial_bridge.serial_write:main',
        ],
    },
)
