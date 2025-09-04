#!/usr/bin/env python3
from __future__ import annotations

import unittest

from rosbridge_library.internal import pngcompression


class TestCompression(unittest.TestCase):
    def test_compress(self) -> None:
        bytes_data = list(range(128)) * 10000
        string = str(bytearray(bytes_data))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        self.assertIsInstance(encoded, str)

    def test_compress_decompress(self) -> None:
        bytes_data = list(range(128)) * 10000
        string = str(bytearray(bytes_data))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        decoded = pngcompression.decode(encoded)
        self.assertEqual(string, decoded)


if __name__ == "__main__":
    unittest.main()
