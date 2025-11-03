from setuptools import setup

package_name = 'map_feedback'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/feedback.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Map evaluation (coverage) + RViz markers',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_feedback_node = map_feedback.map_feedback_node:main',
        ],
    },
)