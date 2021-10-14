========
Overview
========

.. start-badges

.. image:: https://travis-ci.org/mje-nz/python_natnet.svg?branch=master
    :alt: Travis-CI Build Status
    :target: https://travis-ci.org/mje-nz/python_natnet

.. image:: https://ci.appveyor.com/api/projects/status/github/mje-nz/python_natnet?branch=master&svg=true
    :alt: AppVeyor Build Status
    :target: https://ci.appveyor.com/project/mje-nz/python-natnet-pwcxn

.. image:: https://readthedocs.org/projects/python-natnet/badge/?style=flat
    :target: https://readthedocs.org/projects/python-natnet
    :alt: Documentation Status

.. image:: https://codecov.io/github/mje-nz/python_natnet/coverage.svg?branch=master
    :alt: Coverage Status
    :target: https://codecov.io/github/mje-nz/python_natnet

.. end-badges

This is a NatNet client library written in Python, supporting protocol version 3.0.1 (Motive version 2.0).
Regularly rested with protocol 3.0.0 (Motive 2.0).
Reported to work with protocol 3.1.0 (Motive 2.1).
Support for protocol 2.10 in progress (Motive 1.10).

Features:

* Supports rigid bodies and markers
* Skeleton support in progress
* Synchronizes clocks with the server to calculate correct frame timestamps
* Server autodiscovery
* Free software: BSD 3-Clause License

See also:

* `mje-nz/natnet_ros <https://github.com/mje-nz/natnet_ros>`_, a ROS driver based on this library.
* `mje-nz/natnet-sdk-mirror <https://github.com/mje-nz/natnet-sdk-mirror>`_, a versioned mirror of NaturalPoint's NatNet SDK.


Documentation
=============

https://python-natnet.readthedocs.io/


Installation
============

For now, use::

    python setup.py install

to build and install the package, or::

    python setup.py develop

to install in "development mode" i.e. to install symlinks.


Development
===========
Install Python 2.7 and 3.6, git, and pip3 install tox.  On Windows you'll have to manually git to your PATH,
and it's probably best to add at least one Python (tox will use the "py" launcher to find your installed Pythons, but if
you want to be able to run "python27", "pip3" etc on the command line you'll have to set that up yourself).  On Windows,
Gow makes the command line a bit less painful.

To run the checks and tests run::

    tox

To run the most important checks and tests automatically before you commit, run::

    ln -s $(pwd)/pre-commit-hook .git/hooks/pre-commit

