from setuptools import find_packages, setup

package_name = 'tf_transform'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testTransform=tf_transform.only_transform:main',
            'TF2Read=tf_transform.listerner_and_transform:main',
            'OdomRead=tf_transform.odom_tf:main',
        ],
    },
)
