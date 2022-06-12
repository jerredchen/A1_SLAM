from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['a1_slam', 'a1_slam.src.optimization', 'a1_slam.src.registration', 'a1_slam.src.utils'],
    package_dir={'': 'src'}
)
setup(**d)
