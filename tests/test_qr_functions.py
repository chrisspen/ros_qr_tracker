#!/usr/bin/env python
import sys
import unittest

import roslib

PKG = 'ros_qr_tracker'
roslib.load_manifest(PKG)

class Tests(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_qr_functions', Tests)
