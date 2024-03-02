from setuptools import find_packages, setup
import os


package_name = 'show_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/urdf', ['urdf/diff_urdf.urdf']),
        (f'share/{package_name}/rviz', ['rviz/urdf.rviz']),
        (os.path.join('share', package_name), ['launch/show_urdf.launch.py']),
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
        ],
    },
)
