#!/usr/bin/env python
import sys
import unittest

import roslib

PKG = 'ros_qr_tracker'
roslib.load_manifest(PKG)

class Tests(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_qr_node', Tests)
