from setuptools import find_packages
from setuptools import setup

package_name = 'ros_esm_dependencies_generator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ubuntu Robotics',
    author_email='canonical-robotics-community-group@canonical.com',
    maintainer='Ubuntu Robotics',
    maintainer_email='canonical-robotics-community-group@canonical.com',  # noqa: E501
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: GNU General Public License v3.0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'ROS ESM missing dependencies sources generator.'
    ),
    license='GNU General Public License v3.0',
    entry_points={
        'console_scripts': [
            'ros-esm-dependencies-generator = ros_esm_dependencies_generator.ros_esm_dependencies_generator:main',
        ],
    },
)
