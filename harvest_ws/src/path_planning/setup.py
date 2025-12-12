from setuptools import find_packages, setup

package_name = 'path_planning'
submodules = "path_planning/RRT"
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cotrobot',
    maintainer_email='cotrobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "load_obs=path_planning.load_obstacles:main",
             "pub_obs=path_planning.publish_obstacles:main",
            "path_plan=path_planning.find_path:main"
        ],
    },
)
