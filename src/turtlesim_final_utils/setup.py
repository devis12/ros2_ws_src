from setuptools import setup

package_name = 'turtlesim_final_utils'

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
    maintainer='devis',
    maintainer_email='devis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spawn_turtles= turtlesim_final_utils.spawn_turtles:main",
            "controller_turtle= turtlesim_final_utils.controller_turtle:main",
            "field_const= turtlesim_final_utils.field_const",
            "controller_turtle_sol= turtlesim_final_utils.controller_turtle_sol:main",
            "spawn_turtles_sol= turtlesim_final_utils.spawn_turtles_sol:main"
        ],
    },
)
