from setuptools import setup

package_name = 'first_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athens',
    maintainer_email='athens@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'simple_publisher_node = first_package.simple_publisher_node:main',
        'simple_subscriber_node = first_package.simple_subscriber_node:main',
        'simple_pubsub_node = first_package.simple_pubsub_node:main',
        'simple_service = first_package.simple_service:main',
        'simple_service_parameter = first_package.simple_service_parameter:main'
        ],
    },
)
