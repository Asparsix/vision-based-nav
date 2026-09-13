from setuptools import find_packages, setup

package_name = 'ugv_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/segment.launch.py',
            'launch/perception.launch.py',
            'launch/vision_nav.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Live UNet segmentation for UGV camera',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'segmenter = ugv_perception.segmenter:main',
            'seg_costmap = ugv_perception.seg_costmap:main',
            'vision_local_planner = ugv_perception.vision_local_planner:main',
        ],
    },
)
