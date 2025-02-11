from setuptools import find_packages, setup

package_name = 'turtle_control_breno'

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
    maintainer='lcee',
    maintainer_email='gustavobdettogni@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtle_control_breno.turtle_control:main',
            'turtle_client = turtle_control_breno.turtle_client:main',
            'plot_path = turtle_control_breno.plot_path:main',
        ],
    },
)
