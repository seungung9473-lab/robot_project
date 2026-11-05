from glob import glob
from setuptools import setup

pkg = 'soarm_bridge'
setup(
    name=pkg, version='0.2.0', packages=[pkg],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + pkg]),
                ('share/' + pkg, ['package.xml']),
                ('share/' + pkg + '/config', glob('config/*.yaml')),
                ('share/' + pkg + '/launch', glob('launch/*.py')),
                ('share/' + pkg + '/rviz', glob('rviz/*.rviz'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='wanjae', maintainer_email='swj4875@gmail.com',
    description='SO-ARM 4DOF hardware bridge', license='MIT',
    entry_points={'console_scripts': [
        'servo_bridge = soarm_bridge.servo_bridge:main',
        'esp32_bridge = soarm_bridge.esp32_bridge:main',
    ]},
)
