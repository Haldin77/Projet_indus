from setuptools import find_packages, setup

package_name = 'ros2_haply_inverse3_python'

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
    maintainer='pc-rob',
    maintainer_email='pc-rob@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'haply_inverse3_pos = ros2_haply_inverse3_python.haply_inverse3_pos:main',
            'haply_inverse3_quaternion = ros2_haply_inverse3_python.haply_inverse3_quaternion:main',
        ],
    },
)
