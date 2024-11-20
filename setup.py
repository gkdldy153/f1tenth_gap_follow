from setuptools import setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},  # Map the root of the package to 'src'
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [
            'src/gap_follow/scripts/reactive_node.py',
            'src/gap_follow/scripts/reactive_node2.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzangupenn, Hongrui Zheng',
    maintainer_email='zzang@seas.upenn.edu, billyzheng.bz@gmail.com',
    description='f1tenth gap_follow lab',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = gap_follow.scripts.reactive_node:main',
            'reactive_node2 = gap_follow.scripts.reactive_node2:main',
        ],
    },
)

