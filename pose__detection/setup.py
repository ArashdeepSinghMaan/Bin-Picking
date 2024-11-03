from setuptools import find_packages, setup

package_name = 'pose__detection'

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
    maintainer='arashmaan',
    maintainer_email='arashmaan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pose_detection=pose__detection.pose_detection_node:main',
        'pose_estimate=pose__detection.pose_estimate:main',
        'new=pose__detection.new:main',
        'open=pose__detection.ope:main'
        ],
    },
)
