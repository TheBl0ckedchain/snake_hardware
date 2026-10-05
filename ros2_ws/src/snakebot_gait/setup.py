from setuptools import setup

package_name = 'snakebot_gait'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrinal',
    maintainer_email='mrinalxr10@gmail.com',
    description='Serpenoid gait generator for the snake (lateral undulation).',
    license='MIT',
    entry_points={'console_scripts': ['serpenoid = snakebot_gait.serpenoid:main']},
)
