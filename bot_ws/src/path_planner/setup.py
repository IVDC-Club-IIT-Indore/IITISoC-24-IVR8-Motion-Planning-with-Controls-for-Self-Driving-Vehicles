from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manan',
    maintainer_email='manannigam07@gmail.com',
    description='Path planner package using A* and RRT* algorithm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_path_planner = astar_path_planner:main',
            'rrtstar_path_planner = rrtstar_path_planner:main',
        ],
    },
)
