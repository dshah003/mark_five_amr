from setuptools import setup

package_name = 'mark_five_icm20948'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'icm20948_node = mark_five_icm20948.icm20948_node:main',
        ],
    },
)
