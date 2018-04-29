#!/usr/bin/env python
import os

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

CURRENT_DIR = os.path.abspath(os.path.dirname(__file__))

def get_reqs(*fns):
    lst = []
    for fn in fns:
        for package in open(os.path.join(CURRENT_DIR, fn)).readlines():
            package = package.strip()
            if not package:
                continue
            lst.append(package.strip())
    return lst

d = generate_distutils_setup()
print('d0:', d)

d = generate_distutils_setup(
    #version='0.1.0',
    packages=['ros_qr_tracker'],
    package_dir={'': 'src'},
    # scripts=['bin/myscript'],
    install_requires=get_reqs('pip-requirements.txt'),
    #install_requires=['Pillow', 'libzbar-cffi', 'Numpy', 'qrcode'],
)

setup(**d)
