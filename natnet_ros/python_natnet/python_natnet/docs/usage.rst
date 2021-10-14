=====
Usage
=====

To use `natnet` in a project::

	import natnet
	client = natnet.Client.connect()
	client.set_callback(
	    lambda rigid_bodies, markers, timing: print(rigid_bodies))
	client.spin()

This will autodiscover your NatNet server (or complain if there is none or more than one), synchronize clocks, fetch model descriptions, subscribe to mocap frames, and call your callback each time a mocap frame arrives.
For a full example, see ``scripts\natnet-client-demo.py``.
Another example is `mje-nz/natnet_ros <https://github.com/mje-nz/natnet_ros>`_, a ROS driver based on this library.

