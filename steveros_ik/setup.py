import os
from glob import glob
from setuptools import setup

package_name = 'steveros_ik'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'pin-pink',
        'pinocchio',
        'qpsolvers[daqp]',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Parsa Ghasemi',
    maintainer_email='parsa@stevebots.com',
    description='Pink QP-based differential IK for steveros bimanual teleoperation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ik_node = steveros_ik.ik_node:main',
            'xr_bridge = steveros_ik.xr_bridge_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
