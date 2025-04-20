from setuptools import setup

package_name = 'gemini_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/crispe_framework.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='shigemori',
    maintainer_email='yasumasashige790@gmail.com',
    description='ROS2 package for running Gemini service',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemini_service_server = gemini_ros.gemini_service_server:main',
            'gemini_service_client = gemini_ros.gemini_service_client:main',
        ],
    },
)