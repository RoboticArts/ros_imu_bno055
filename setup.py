from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ros_imu_bno055'],
    package_dir={'': 'include'},
)

setup(**setup_args)
