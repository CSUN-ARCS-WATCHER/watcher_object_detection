from setuptools import find_packages, setup

package_name = 'arcs_positional_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/tracking.launch.py']),
    # remove this for now:
    # ('share/' + package_name + '/rviz', ['rviz/pos_tracking_config.rviz']),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'positional_tracking_node = arcs_positional_tracking.positional_tracking_node:main',
        'fake_pose_pub = arcs_positional_tracking.fake_pose_pub:main',
    ],
},

)
