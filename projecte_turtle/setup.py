from setuptools import find_packages, setup

package_name = 'projecte_turtle'

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
    maintainer='albert',
    maintainer_email='albert.ardiaca@students.salle.url.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'controller_tortuga = projecte_turtle.controller_tortuga:main',
            'paisatje_paint = projecte_turtle.paisatje_paint:main',
        ],
    },
)
