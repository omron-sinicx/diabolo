#!/usr/bin/env python
# coding: utf-8

from __future__ import absolute_import, print_function

import io
import re
from glob import glob
from os.path import basename, dirname, join, splitext

from setuptools import find_packages, setup


def read(*names, **kwargs):
    return io.open(
        join(dirname(__file__), *names),
        encoding=kwargs.get('encoding', 'utf8')
    ).read()


setup(
    name='natnet',
    version='0.1.0',
    license='BSD 3-Clause License',
    description='NatNet 3 client',
    long_description=re.compile('^.. start-badges.*^.. end-badges', re.M | re.S)
            .sub('', read('README.rst')),
    author='Matthew Edwards',
    author_email='matthew@matthewedwards.co.nz',
    url='https://github.com/mje-nz/python_natnet',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    py_modules=[splitext(basename(path))[0] for path in glob('src/*.py')],
    include_package_data=True,
    zip_safe=False,
    classifiers=[
        # complete classifier list: http://pypi.python.org/pypi?%3Aaction=list_classifiers
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Operating System :: MacOS',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: Implementation :: CPython'
    ],
    keywords='natnet NaturalPoint Optitrack motion capture mocap',
    install_requires=[
        'attrs>=15.2'
    ],
    extras_require={
        ':python_version<"3.5"': ['typing'],
        ':python_version<"3.4"': ['enum34']
    }
)
