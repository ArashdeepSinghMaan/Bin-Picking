from setuptools import setup

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'torch', 'opencv-python'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='arashmaa@example.com',
    description='ROS2 node for YOLOv5 inference on RGB camera images',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_node = yolo.yolov5:main'
        ],
    },
)

