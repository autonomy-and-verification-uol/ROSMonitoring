import oracle

PROPERTY = "once[0:3]{front_distance}"
predicates = dict(
    time = 0,
    front_distance = False
)
def abstract_message(message):
    if message['ranges'][360] >= 1.5:
        predicates['front_distance'] = True
    else:
        predicates['front_distance'] = False
    predicates['time'] = message['time']
    return predicates
