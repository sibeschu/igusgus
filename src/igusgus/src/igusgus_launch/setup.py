import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'igusgus_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[# ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')) ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrl-linux24-04',
    maintainer_email='amrl-linux24-04@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
