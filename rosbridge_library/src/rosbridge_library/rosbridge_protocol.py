from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
# import for defragmentation
from rosbridge_library.capabilities.defragmentation import Defragment

# uncomment to enable Fragmentation
#from rosbridge_library.capabilities.fragmentation import Fragmentation


class RosbridgeProtocol(Protocol):
    """ Adds the handlers for the rosbridge opcodes """
    rosbridge_capabilities = [CallService, Advertise, Publish, Subscribe, Defragment]

    print "registered capabilities:"
    for cap in rosbridge_capabilities:
        print "-", str(cap)

    def __init__(self, client_id):
        Protocol.__init__(self, client_id)
        for capability_class in self.rosbridge_capabilities:
            self.add_capability(capability_class)
