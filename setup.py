from setuptools import setup
from glob import glob
import os

package_name = 'ros2_performance'

setup(
    # Information
    name=package_name,
    version='0.0.0',
    author='Team',
    author_email='',
    description='Core core',
    license='Proprietary',
    # Content
    packages=[package_name],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'publisher = ros2_performance.publisher:main',
            'subscriber = ros2_performance.subscriber:main',
        ],
    },
    # Requirements
    setup_requires=[
    ],
    tests_require=[],
)
