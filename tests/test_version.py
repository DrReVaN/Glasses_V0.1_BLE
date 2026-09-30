import hashlib
import importlib.util
from pathlib import Path
import struct
import unittest
import zlib

TOOLS=Path(__file__).resolve().parents[1]/'tools'
spec=importlib.util.spec_from_file_location('version',TOOLS/'version.py')
version=importlib.util.module_from_spec(spec); spec.loader.exec_module(version)

class VersionTests(unittest.TestCase):
    def test_order_is_numeric(self):
        self.assertGreater(version.parse('0.10.0'),version.parse('0.9.12'))
        self.assertGreater(version.parse('1.0.0'),version.parse('0.65535.65535'))
        self.assertGreaterEqual(version.parse(version.current()),(0,3,0))

    def test_noncanonical_or_overflow_rejected(self):
        for text in ('1.0','v1.0.0','01.0.0','1.-1.0','1.0.0-beta','65536.0.0',None):
            with self.assertRaises(ValueError): version.parse(text)

    def test_version_is_bound_to_binary(self):
        image=bytearray(0x200)
        image[0x140:0x14c]=b'SGV1'+struct.pack('<HHHBB',0,3,0,1,0)
        version.validate_identity(image,'0.3.0')
        for text in ('0.3.1','1.0.0'):
            with self.assertRaises(ValueError): version.validate_identity(image,text)
        with self.assertRaises(ValueError): version.validate_identity(image,'0.3.0',1)
        image[0x140]=0
        with self.assertRaises(ValueError): version.validate_identity(image,'0.3.0')
        version.validate_identity(image,'0.2.0')

if __name__=='__main__': unittest.main()
