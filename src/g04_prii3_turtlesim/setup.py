from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'g04_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

         # Instalamos los archivos de la carpeta launch
         (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlosubnt2',
    maintainer_email='carlosubnt2@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dibujo04 = g04_prii3_turtlesim.dibujo04:main',
        ],
    },
)
