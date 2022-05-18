from setuptools import setup

package_name = 'static_pose_injector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'lxml'],
    zip_safe=True,
    maintainer='Chris Brammer',
    maintainer_email='chris.brammer@extend3d.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster = static_pose_injector.broadcaster:main',
            'camerainfo_projector = static_pose_injector.camerainfo_projector:main'
        ],
    },
)
