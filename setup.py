from setuptools import find_packages, setup

package_name = 'gemini_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/gemini_llm_launch.py',
            'launch/gemini_stt_launch.py',
            'launch/gemini_vlm_launch.py',
        ]),
    ],
    install_requires=['setuptools', 'gemini_interface'],
    zip_safe=False,
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='ROS2 package for running Gemini scripts',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gemini_llm = gemini_ros.gemini_llm:main',
            'gemini_stt = gemini_ros.gemini_stt:main',
            'gemini_vlm = gemini_ros.gemini_vlm:main',
        ],
    },
)