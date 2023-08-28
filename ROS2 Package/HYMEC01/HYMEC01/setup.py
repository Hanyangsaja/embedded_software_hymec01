from setuptools import find_packages, setup

package_name = 'HYMEC01.1'

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
    maintainer='wonker',
    maintainer_email='wonker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'laptop = HYMEC01.laptop:main',
        'robot1 = HYMEC01.robot1:main',
        'robot2 = HYMEC01.robot2:main',
        'robot3 = HYMEC01.robot3:main',
        'robot4 = HYMEC01.robot4:main',
        ],
    },
)
