import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tennis_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 告訴編譯器：把 launch 裡面的 .py 檔全部包進去
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 告訴編譯器：把 urdf 裡面的 .urdf 檔全部包進去
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # 告訴編譯器：把 urdf 裡面的 .sdf 檔 (網球) 全部包進去
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.sdf')),
        # 告訴編譯器把 meshes 裡面的 .stl 檔全部包進去
        (os.path.join('share', package_name, 'urdf', 'meshes'), glob('urdf/meshes/*.stl')),
        # ↓↓↓ 請加入這一行，讓系統編譯時搬運 world 檔案 ↓↓↓
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sean',
    maintainer_email='y31724005@gmail.com',
    description='Tennis ball picking robot POC',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = tennis_bot.vision_node:main',
            'control_node = tennis_bot.control_node:main',
            'patrol_node = tennis_bot.patrol_node:main',
        ],
    },
)