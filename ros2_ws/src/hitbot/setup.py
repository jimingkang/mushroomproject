from setuptools import setup

package_name = 'hitbot'

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
    maintainer='dev_jung',
    maintainer_email='seedn7777@cwsfa.co.kr',
    description='Hitbot Z-arm scara robot',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hitbot_controller        = hitbot.hitbot_controller:main',
            'hitbot_control_pub       = hitbot.hitbot_control_pub:main',
        ],
    },
)
