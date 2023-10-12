#!/usr/bin/env python
import unittest

from rosbridge_library.internal import pngcompression


class TestCompression(unittest.TestCase):
    def test_compress(self):
        bytes_data = list(range(128)) * 10000
        string = str(bytearray(bytes_data))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)

    def test_compress_decompress(self):
        bytes_data = list(range(128)) * 10000
        string = str(bytes(bytes_data))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        decoded = pngcompression.decode(encoded)
        self.assertEqual(string, decoded)
