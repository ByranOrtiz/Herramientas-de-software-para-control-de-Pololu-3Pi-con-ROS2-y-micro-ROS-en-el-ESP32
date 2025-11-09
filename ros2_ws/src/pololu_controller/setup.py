from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pololu_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Archivos base del paquete
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Archivos de configuración YAML
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

        # Archivos de lanzamiento (si existen)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryan',
    maintainer_email='bryanred431@gmail.com',
    description='Point-to-point controller for Pololu 3pi with exponential PID and multi-agent support',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cp2p_pololu = pololu_controller.cp2p_pololu:main',
            'multi_agent_controller = pololu_controller.multi_agent_controller:main',
            'demo_chain_follower = pololu_controller.demo_chain_follower:main',
        ],
    },
)
