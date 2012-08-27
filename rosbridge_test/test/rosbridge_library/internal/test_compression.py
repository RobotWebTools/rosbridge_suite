#!/usr/bin/env python
PKG = 'rosbridge_library'
import roslib; roslib.load_manifest(PKG)
import rospy

from rosbridge_library.internal import pngcompression

import unittest


class TestCompression(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_compression")

    def test_compress(self):
        bytes = range(128) * 10000
        string = str(bytearray(bytes))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        
    def test_compress_decompress(self):
        bytes = range(128) * 10000
        string = str(bytearray(bytes))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        decoded = pngcompression.decode(encoded)
        self.assertEqual(string, decoded)
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_compression', TestCompression)
