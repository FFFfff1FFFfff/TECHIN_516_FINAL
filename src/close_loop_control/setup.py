from setuptools import find_packages, setup

package_name = 'close_loop_control'

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
    maintainer='yifanli8',
    maintainer_email='yifanli8@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'close_loop_odom = close_loop_control.close_loop_odom:main',
            'turtle_final = close_loop_control.turtle_final:main',
            'turtle_final_pro = close_loop_control.turtle_final_pro:main',
            'turtle_final_pro_back = close_loop_control.turtle_final_pro_back:main',
            'turtle_final_pro_bf = close_loop_control.turtle_final_pro_bf:main',
            'turtle_final_bf = close_loop_control.turtle_final_bf:main',
        ],
    },
)
