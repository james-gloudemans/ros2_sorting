from setuptools import setup

package_name = 'sorting'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='james.gloudemans@gmail.com',
    description='Scripts for autonomous sorting with mobile robot.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'picker = sorting.sorter:main',
            'object_selector = sorting.object_selector:main',
            'picker = sorting.picker:main',
            'placer = sorting.placer:main',
        ],
    },
)
