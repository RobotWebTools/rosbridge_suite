#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest

from rosbridge_library.internal import pngcompression


class TestCompression(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_compression")

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

PKG = 'rosbridge_library'
NAME = 'test_compression'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestCompression)

