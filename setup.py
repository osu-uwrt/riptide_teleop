from setuptools import setup

package_name = 'riptide_teleop2'

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
    maintainer='hayden',
    maintainer_email='hgray576@gmail.com',
    description='Package used for the teleoperation of the team\'s robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps3_teleop = riptide_teleop2.ps3_teleop:main'
        ],
    },
)
