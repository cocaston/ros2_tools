import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bag_to_file'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caston',
    maintainer_email='castoner@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_to_file_node = bag_to_file.app:main',
        ],
    },
)
