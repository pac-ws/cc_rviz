from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cc_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saurav',
    maintainer_email='sauravag@upenn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'idf = cc_rviz.idf:main'
        ],
    },
)
