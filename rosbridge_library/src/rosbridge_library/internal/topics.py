""" Exceptions and code common to both publishers and subscribers """


class TopicNotEstablishedException(Exception):
    def __init__(self, topic):
        Exception.__init__(self,
        "Cannot infer topic type for topic %s as it is not yet advertised" %
        (topic,))


class TypeConflictException(Exception):
    def __init__(self, topic, orig_type, new_type):
        Exception.__init__(self,
        ("Tried to register topic %s with type %s but it is already" +
        " established with type %s") % (topic, new_type, orig_type))
