from setuptools import find_packages, setup

package_name = 'ros_april_tag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'config/tags.yaml',
                                   'launch/detect_tag.launch.xml',
                                   'config/rviz.config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jialuyu',
    maintainer_email='jialuyu2024@u.northwestern.edu',
    description='Testing for April Tag',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [     
        ],
    },
)
