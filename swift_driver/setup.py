from setuptools import find_packages, setup

package_name = 'swift_driver'

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
    maintainer='arunser',
    maintainer_email='stormbreaker.004@gmail.com',
    description='The swift_driver package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lum_drone = swift_driver.driver:main',
        ],
    },
)
