from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['spot_ros_interface'],
    scripts=['scripts/spot_ros_interface.py'],
    package_dir={'': 'src'}
)

setup(**d)