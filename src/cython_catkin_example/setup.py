from setuptools import setup
from distutils.sysconfig import get_python_lib
import glob
import os
import sys

if os.path.exists('readme.rst'):
    print("""The setup.py script should be executed from the build directory. Please see the file 'readme.rst' for further instructions.""")
    sys.exit(1)


setup(
    name = "cython_catkin_example",
    package_dir = {'': 'src'},
    data_files = [(get_python_lib(), glob.glob('src/*.so'))
                  #,('bin', ['bin/cython_catkin_example'])
                  ],
    author = 'Marco Esposito',
    description = 'Example of Cython and catkin integration',
    license = 'Apache',
    keywords = 'cmake cython build',
    url = 'http://github.com/marcoesposito1988/cython_catkin_example',
    zip_safe = False,
    )
