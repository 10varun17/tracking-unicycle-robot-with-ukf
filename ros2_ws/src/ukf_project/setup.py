from setuptools import find_packages, setup

package_name = 'ukf_project'

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
    maintainer='varuntemp',
    maintainer_email='17varun.rayamajhi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = ukf_project.controller:main",
            "robot = ukf_project.robot_node:main",
            "sensor = ukf_project.sensor_node:main",
            "ukf = ukf_project.ukf_node:main"
        ],
    },
)
