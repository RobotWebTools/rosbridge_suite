from rosbridge_library.internal.cbor_conversion import extract_cbor_values
from rosbridge_library.internal.message_conversion import (
    extract_values as extract_json_values,
)

try:
    from cbor import dumps as encode_cbor
except ImportError:
    from rosbridge_library.util.cbor import dumps as encode_cbor


class OutgoingMessage:
    """A message wrapper for caching encoding operations."""

    def __init__(self, message):
        self._message = message
        self._json_values = None
        self._cbor_values = None
        self._cbor_msg = None
        self._cbor_raw_msg = None

    @property
    def message(self):
        return self._message

    def get_json_values(self):
        if self._json_values is None:
            self._json_values = extract_json_values(self._message)
        return self._json_values

    def get_cbor_values(self):
        if self._cbor_values is None:
            self._cbor_values = extract_cbor_values(self._message)
        return self._cbor_values

    def get_cbor(self, outgoing_msg):
        if self._cbor_msg is None:
            outgoing_msg["msg"] = self.get_cbor_values()
            self._cbor_msg = encode_cbor(outgoing_msg)

        return self._cbor_msg

    def get_cbor_raw(self, outgoing_msg):
        if self._cbor_raw_msg is None:
            self._cbor_raw_msg = encode_cbor(outgoing_msg)

        return self._cbor_raw_msg
