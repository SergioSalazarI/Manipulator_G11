from setuptools import setup

package_name = 'turtle_bot_nodos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergio',
    maintainer_email='davidisairias24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_manipulator_teleop= turtle_bot_nodos.robot_manipulator_teleop:main',
            'robot_manipulator_maker = turtle_bot_nodos.robot_manipulator_maker:main',
            'turtle_bot_player = turtle_bot_nodos.turtle_bot_player:main',
            'turtle_bot_save_route = turtle_bot_nodos.turtle_bot_save_route:main',
            'turtle_bot_position = turtle_bot_nodos.turtle_bot_position:main'#,
            #'robot_hearer = turtle_bot_nodos.robot_hearer:main'
        ],
    },
)
