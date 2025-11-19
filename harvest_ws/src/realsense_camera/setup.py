from setuptools import find_packages, setup

package_name = 'realsense_camera'

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
            "run_camera=realsense_camera.camera:main",
            "run_camera2=realsense_camera.two_camera:main",
            "test_camera=realsense_camera.test:main"
        ],
    },
)
