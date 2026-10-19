from setuptools import setup

package_name = 'snakebot_gait'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nn_policy.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrinal',
    maintainer_email='mrinalxr10@gmail.com',
    description='Serpenoid gait generator, keyboard teleop and NN navigation policy for the snake.',
    license='MIT',
    entry_points={'console_scripts': [
        'serpenoid = snakebot_gait.serpenoid:main',
        'teleop_key = snakebot_gait.teleop_key:main',
        'nn_controller = snakebot_gait.nn_controller:main',
    ]},
)
