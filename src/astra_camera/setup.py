from setuptools import setup

package_name = 'astra_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'modules'],
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
            'astra=astra_camera.rostra:main'
        ],
    },
)