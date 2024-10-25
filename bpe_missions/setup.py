from setuptools import find_packages, setup

package_name = 'bpe_missions'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sander Doodeman',
    maintainer_email='s.doodeman@tue.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission=bpe_missions.mission:main',
            'land=bpe_missions.land:main'
        ],
    },
)