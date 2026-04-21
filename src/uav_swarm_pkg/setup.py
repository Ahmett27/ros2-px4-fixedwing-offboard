from setuptools import setup
import os                  
from glob import glob

package_name = 'uav_swarm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # YENİ EKLENDİ: Launch klasöründeki tüm .py dosyalarını share dizinine kopyala
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmet',
    maintainer_email='ahmetemirhanates1@gmail.com',
    description='UAV Swarm Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fixed_wing_control = uav_swarm_pkg.fixed_wing_control:main',
            'suru_kontrol = uav_swarm_pkg.suru_iha:main',
        ],
    },
)
