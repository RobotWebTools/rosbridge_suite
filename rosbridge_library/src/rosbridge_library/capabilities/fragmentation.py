from rosbridge_library.capability import Capability


class Fragmentation(Capability):
    """ The Fragmentation capability doesn't define any incoming operation
    handlers, but provides methods to fragment outgoing messages """
    
    fragmentation_seed = 0

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

    def fragment(self, message, fragment_size, mid=None):
        """ Serializes the provided message, then splits the serialized
        message according to fragment_size, then sends the fragments.
        
        If the size of the message is less than the fragment size, then
        the original message is returned rather than a single fragment
        
        Since fragmentation is typically only used for very large messages,
        this method returns a generator for fragments rather than a list
        
        Keyword Arguments
        message       -- the message dict object to be fragmented
        fragment_size -- the max size for the fragments
        mid           -- (optional) if provided, the fragment messages
        will be given this id.  Otherwise an id will be auto-generated.

        Returns a generator of message dict objects representing the fragments
        """
        # All fragmented messages need an ID so they can be reconstructed
        if mid is None:
            mid = self.fragmentation_seed
            self.fragmentation_seed = self.fragmentation_seed + 1
            
        serialized = self.protocol.serialize(message, mid)
        
        if serialized is None:
            return []
        
        message_length = len(serialized)
        if message_length <= fragment_size:
            return [message]
        
        return self._fragment_generator(serialized, fragment_size, mid)
    
    def _fragment_generator(self, msg, size, mid):
        """ Returns a generator of fragment messages """
        total = ((len(msg)-1) / size) + 1
        n = 0
        for i in range(0, len(msg), size):
            fragment = msg[i:i+size]
            yield self._create_fragment(fragment, n, total, mid)
            n = n + 1
    
    def _create_fragment(self, fragment, num, total, mid):
        """ Given a string fragment of the original message, creates
        the appropriate fragment message """
        return {
            "op": "fragment",
            "id": mid,
            "data": fragment,
            "num": num,
            "total": total
        }