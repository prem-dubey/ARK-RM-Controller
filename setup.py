from setuptools import find_packages, setup

package_name = 'review'

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
    maintainer='piyush',
    maintainer_email='piyush.10981@gmail.com',
    description='For Review meet ',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'listener_ekf = review.ekf:main',
            'listener_pid = review.pid:main'
        ],
    },
)
