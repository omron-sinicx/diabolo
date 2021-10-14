# coding: utf-8
"""Parse and pretty-print a MocapFrame, then time how long it takes."""

import attr
import collections
import yaml
from optitrack.protocol import deserialize

# TODO: Put this pretty-printing code somewhere

# https://stackoverflow.com/a/8661021
represent_dict_order = lambda self, data: self.represent_mapping('tag:yaml.org,2002:map',
                                                                 data.items())
yaml.add_representer(collections.OrderedDict, represent_dict_order)

data = open('../tests/mocapframe_packet_v3.bin', 'rb').read()
frame = deserialize(data)
print(yaml.dump(attr.asdict(frame, dict_factory=collections.OrderedDict), default_flow_style=False))

import timeit

n = 10000
t = timeit.timeit('deserialize(data)', globals=locals(), number=n)
print('Parsing time: {} us'.format(t/n*1e6))
