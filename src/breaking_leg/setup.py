from setuptools import setup

package_name = 'breaking_leg'

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
    maintainer='tx2',
    maintainer_email='tx2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_control=breaking_leg.get_control:main',
            'hand_control=breaking_leg.hand_control:main',
            'tracking=breaking_leg.tracking:main'
        ],
    },
)
