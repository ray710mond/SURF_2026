from setuptools import find_packages, setup

package_name = 'surf_data_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'data_tracker = surf_data_tracker.tracker:main',
        'adaptive_mode_report = surf_data_tracker.adaptive_mode_report:main',
    ]},
)
