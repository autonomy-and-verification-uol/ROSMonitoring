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
PROPERTY = "pre{safety_distance: true}"

# predicates used in the property (initialization for time 0)
predicates = dict()
# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    predicates['safety_distance'] = message['distance_robot_human_operator'] >= message['human_operator_speed'] * (0.1 + 0.5) + message['robot_speed'] * 0.1 + 0 + 2.5
    print('Current distance: ' + str(message['distance_robot_human_operator']) + ' [m]')
    print('Minimum distance according to ISO15066: ' + str(message['human_operator_speed'] * (0.1 + 0.5) + message['robot_speed'] * 0.1 + 0 + 2.5) + ' [m]')
    print('Robot\'s speed must be <= ' + str((message['distance_robot_human_operator'] - (message['human_operator_speed'] * (0.1 + 0.5)) - 2.5) / 0.1) + ' [m/s]')
    print('')
    return predicates
# This function has to be defined by the user depending on the property defined.
# In this case we have just implemented a simple and general function which
# updates the predicates if it finds the topic in the list of predicates.
# Since the property is defined on predicates, we need this function to update the
# predicates each time a message is observed. This abstraction of course is totally
# dependent on the specific application.
