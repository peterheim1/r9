from setuptools import setup

package_name = 'r2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petrus',
    maintainer_email='petrus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino = r2_bringup.arduino:main',
            'head = r2_bringup.head:main',
            'joint_state = r2_bringup.joint_state:main',
            'voice_serv = r2_bringup.voice_serv:main',
            'arm_driver = r2_bringup.arm_driver:main'
        ],
    },
)
