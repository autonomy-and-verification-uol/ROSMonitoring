import oracle

# type of the property (PTL, MTL, or STL)
TYPE = oracle.TypeOfProperty.MTL

# MTL property to verify
PROPERTY = "(historically[0:5]((door_open) and not dow_suppressed) or (not chatter)) -> door_open_warning"

# predicates used in the property (initialization for time 0)
predicates = dict(
    time = 0,
    door_open = False,
    dow_suppressed = False,
    door_open_warning = False,
    chatter = False
)

# function to abstract a ROS message into a list of predicates
def abstract_message(message):
    if message['topic'] in predicates:
        if isinstance(message['data'], bool):
            predicates[message['topic']] = message['data']
        else:
            predicates[message['topic']] = False
    predicates['time'] = int(message['time'])
    print("ROS monitor said: %s" % (predicates))
    return predicates
