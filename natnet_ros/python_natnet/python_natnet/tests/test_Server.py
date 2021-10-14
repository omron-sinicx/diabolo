# coding: utf-8
"""Integration tests for comms module using Server class."""


import sys
import time

import pytest

import natnet

if sys.platform == 'win32':
    # For some reason multiprocessing is letting me pickle lambdas everywhere except on Windows.
    # The 'multiprocess' library from pathos uses dill instead of pickle, so happily pickles
    # everything, but doesn't have coverage support.
    # So, use multiprocess on Windows and multiprocessing elsewhere, and let Codecov sort it out
    import multiprocess as multiprocessing
else:
    import multiprocessing


class MPServer(natnet.Server):

    def __init__(self, started_event, exit_event, *args, **kwargs):
        super(MPServer, self).__init__(*args, **kwargs)
        self.started_event = started_event  # type: multiprocessing.Event
        self.exit_event = exit_event  # type: multiprocessing.Event

    def _run(self, *args, **kwargs):
        self.started_event.set()
        super(MPServer, self)._run(*args, **kwargs)

    def should_exit(self):
        return self.exit_event.is_set()
    should_exit = property(should_exit, lambda self, e: None)


@pytest.fixture()
def server():
    started_event = multiprocessing.Event()
    exit_event = multiprocessing.Event()
    process = multiprocessing.Process(target=lambda: MPServer(started_event, exit_event).run(rate=1000))
    process.start()
    started_event.wait()  # Starting processes is really slow on Windows
    time.sleep(0.1)  # Give the server a head start at stdout
    yield
    exit_event.set()
    process.join(timeout=1)
    process.terminate()


@pytest.mark.timeout(5)
def test_autodiscovery(server):
    c = natnet.Client.connect(timeout=1)
    c.run_once()
