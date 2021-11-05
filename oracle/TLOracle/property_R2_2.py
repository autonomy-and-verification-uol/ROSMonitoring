# MIT License
#
# Copyright (c) [2020] [Angelo Ferrando]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import oracle

# property to verify
PROPERTY = r'historically(({isTargetGrasped: true} -> {dx < 0.1, dy < 0.1, dz < 0.1, other_distances: true}) and ({trigger: false} -> {other_distances: true}))'

# predicates used in the property (initialization for time 0)
predicates = dict(
)
# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    if 'd1' in message and 'd2' in message and 'd3' in message and 'd4' in message and 'd5' in message and 'd6' in message and 'd7' in message and 'd8' in message:
        predicates['dx'] = message['d8']['x']
        predicates['dy'] = message['d8']['y']
        predicates['dz'] = message['d8']['z']
        predicates['other_distances'] = (message['d1']['x'] != 0 or message['d1']['y'] != 0 or message['d1']['z'] != 0) and (message['d2']['x'] != 0 or message['d2']['y'] != 0 or message['d2']['z'] != 0) and (message['d3']['x'] != 0 or message['d3']['y'] != 0 or message['d3']['z'] != 0) and (message['d4']['x'] != 0 or message['d4']['y'] != 0 or message['d4']['z'] != 0) and (message['d5']['x'] != 0 or message['d5']['y'] != 0 or message['d5']['z'] != 0) and (message['d6']['x'] != 0 or message['d6']['y'] != 0 or message['d6']['z'] != 0) and (message['d7']['x'] != 0 or message['d7']['y'] != 0 or message['d7']['z'] != 0)
    if 'isTargetGrasped' in message:
        predicates['isTargetGrasped'] = message['isTargetGrasped']
    return predicates
# This function has to be defined by the user depending on the property defined.
# In this case we have just implemented a simple and general function which
# updates the predicates if it finds the topic in the list of predicates.
# Since the property is defined on predicates, we need this function to update the
# predicates each time a message is observed. This abstraction of course is totally
# dependent on the specific application.
