from setuptools import find_packages, setup

package_name = 'my_new_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', ['launch/sort_world.launch.py']),
   	('share/' + package_name + '/worlds', ['worlds/sort_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yifanli8',
    maintainer_email='yifanli8@uw.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'trajectory_from_csv = my_new_trajectory.trajectory_from_csv:main',
		'gen3lite_pymoveit2 = my_new_trajectory.gen3lite_pymoveit2:main',
		# 'sort_task = my_new_trajectory.sort_task:main',
		# 'pick_place = my_new_trajectory.pick_place:main',
		# 'run_real_traje = my_new_trajectory.run_real_traje:main',
		'run_real_grasp = my_new_trajectory.run_real_grasp:main',
		'run_real_final = my_new_trajectory.run_real_final:main'
        ],
    },
)
