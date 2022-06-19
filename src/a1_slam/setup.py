from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'a1_slam',
        'nodes',
        'registration'
    ],
    package_dir={'': 'src'}
)
setup(**d)
