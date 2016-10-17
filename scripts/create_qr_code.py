#!/usr/bin/env python
"""
Usage:

    create_qr_code.py "some text" myqrcode.jpg

"""
from __future__ import print_function
import sys
import qrcode

if __name__ == '__main__':
    
    data = sys.argv[1]
    fn = sys.argv[2]
    img = qrcode.make(data)
    img.save(fn, 'JPEG')
    print('Wrote image:', fn, file=sys.stderr)
