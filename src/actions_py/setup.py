from setuptools import find_packages, setup

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='devis.dalmoro@eurecat.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        [
            "count_until_server = actions_py.count_until_server:main",
            "count_until_client = actions_py.count_until_client:main",

            "robot_move_server = actions_py.robot_move_server:main",
            "robot_move_client = actions_py.robot_move_client:main"
        ],
    },
)
