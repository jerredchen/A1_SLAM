#!/usr/bin/env python

import platform
import sys
from codecs import open  # To use a consistent encoding
from os import path

# Always prefer setuptools over distutils
from setuptools import find_packages, setup

here = path.abspath(path.dirname(__file__))

setup(
    name="A1_SLAM",
    version="1.0.0",
    description="",
    author="Georgia Institute of Technology",
    license="MIT",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX",
        "Operating System :: MacOS",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    keywords="robotics",
    packages=['optimization', 'utils', 'registration'],
    python_requires=">= 3.5",
    install_requires=[
        "pytest"
    ]
)