from setuptools import find_packages, setup

package_name = 'node_topics_py'

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
    maintainer='robousr',
    maintainer_email='erik.pena@ingenieria.unam.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node_py = node_topics_py.my_node_py:main',
            'pub_node_py = node_topics_py.pub_py:main',
            'sub_counter_py = node_topics_py.sub_counter_py:main',
            'mobile_controller_py = node_topics_py.mobile_control_py:main',
            'mobile_test_py = node_topics_py.mobile_test_py:main',
            'scara_test_py = node_topics_py.scara_test_py:main',
            'scara_tray_line_py = node_topics_py.scara_tray_line_py:main'
        ],
    },
)
