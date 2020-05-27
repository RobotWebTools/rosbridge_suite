import struct
import sys

PYTHON2 = sys.version_info < (3, 0)

try:
    from cbor import Tag
except ImportError:
    from rosbridge_library.util.cbor import Tag


LIST_TYPES = [list, tuple]
INT_TYPES = ['byte', 'char', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
FLOAT_TYPES = ['float32', 'float64']
STRING_TYPES = ['string']
BOOL_TYPES = ['bool']
TIME_TYPES = ['time', 'duration']
BOOL_ARRAY_TYPES = ['bool[]']
BYTESTREAM_TYPES = ['uint8[]', 'char[]']

# Typed array tags according to <https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00>
# Always encode to little-endian variant, for now.
TAGGED_ARRAY_FORMATS = {
    'uint16[]': (69, '<{}H'),
    'uint32[]': (70, '<{}I'),
    'uint64[]': (71, '<{}Q'),
    'byte[]': (72, '{}b'),
    'int8[]': (72, '{}b'),
    'int16[]': (77, '<{}h'),
    'int32[]': (78, '<{}i'),
    'int64[]': (79, '<{}q'),
    'float32[]': (85, '<{}f'),
    'float64[]': (86, '<{}d'),
}


def extract_cbor_values(msg):
    """Extract a dictionary of CBOR-friendly values from a ROS message.

    Primitive values will be casted to specific Python primitives.

    Typed arrays will be tagged and packed into byte arrays.
    """
    out = {}
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        val = getattr(msg, slot)

        if PYTHON2:
            slot = unicode(slot)  # noqa: F821

        # string
        if slot_type in STRING_TYPES:
            out[slot] = unicode(val) if PYTHON2 else str(val)  # noqa: F821

        # bool
        elif slot_type in BOOL_TYPES:
            out[slot] = bool(val)

        # integers
        elif slot_type in INT_TYPES:
            out[slot] = int(val)

        # floats
        elif slot_type in FLOAT_TYPES:
            out[slot] = float(val)

        # time/duration
        elif slot_type in TIME_TYPES:
            out[slot] = {
                'secs': int(val.secs),
                'nsecs': int(val.nsecs),
            }

        # byte array
        elif slot_type in BYTESTREAM_TYPES:
            if PYTHON2:
                out[slot] = bytes(bytearray(val))
            else:
                out[slot] = bytes(val)

        # bool array
        elif slot_type in BOOL_ARRAY_TYPES:
            out[slot] = [bool(i) for i in val]

        # numeric arrays
        elif slot_type in TAGGED_ARRAY_FORMATS:
            tag, fmt = TAGGED_ARRAY_FORMATS[slot_type]
            fmt_to_length = fmt.format(len(val))
            packed = struct.pack(fmt_to_length, *val)
            out[slot] = Tag(tag=tag, value=packed)

        # array of messages
        elif type(val) in LIST_TYPES:
            out[slot] = [extract_cbor_values(i) for i in val]

        # message
        else:
            out[slot] = extract_cbor_values(val)

    return out
