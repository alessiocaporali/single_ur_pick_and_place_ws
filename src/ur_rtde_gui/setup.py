from setuptools import find_packages, setup

package_name = 'ur_rtde_gui'

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
    maintainer='alessio',
    maintainer_email='alessio.caporali5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur_rtde_gui = ur_rtde_gui.ur_rtde_gui_node:main',
            'test_moveJ = ur_rtde_gui.test_moveJ:main',
        ],
    },

)
