import unittest

from rosbridge_library.protocol import has_binary


class TestProtocol(unittest.TestCase):
    def test_has_binary(self) -> None:
        self.assertTrue(has_binary({"data": bytes([1, 2, 3])}))
        self.assertTrue(has_binary({"data": bytearray([1, 2, 3])}))
        self.assertTrue(has_binary(bytes([1, 2, 3])))
        self.assertTrue(has_binary(bytearray([1, 2, 3])))


if __name__ == "__main__":
    unittest.main()
