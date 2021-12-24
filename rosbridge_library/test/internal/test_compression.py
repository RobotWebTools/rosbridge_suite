#!/usr/bin/env python
import sys
import rospy
import rosunit
import unittest

from rosbridge_library.internal import pngcompression


PKG = 'rosbridge_library'
NAME = 'test_compression'


class TestCompression(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)

    def test_compress(self):
        bytes = list(range(128)) * 10000
        string = str(bytearray(bytes))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)

    def test_compress_decompress(self):
        bytes = list(range(128)) * 10000
        string = str(bytearray(bytes))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        decoded = pngcompression.decode(encoded)
        self.assertEqual(string, decoded)


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestCompression)

