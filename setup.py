from setuptools import find_packages, setup

package_name = 'load_cell_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    ('share/' + package_name + '/launch',
        ['launch/nav2_load_delivery.launch.py']),
    ('share/' + package_name + '/config',
        ['config/load_cell_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jd',
    maintainer_email='ghanshyamjadhav755@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'load_cell_delivery = load_cell_pkg.load_cell_delivery_node:main',
            'load_cell_data = load_cell_pkg.load_cell_data:main',
        ],
    },
)
