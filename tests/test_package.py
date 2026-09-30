import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
import hashlib
import zlib
spec=importlib.util.spec_from_file_location("package",Path(__file__).resolve().parents[1]/"tools/package.py")
package=importlib.util.module_from_spec(spec);spec.loader.exec_module(package)
class PackageTests(unittest.TestCase):
    def test_manifest_and_vectors(self):
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/"app.bin";manifest=binary.with_suffix(".json")
            data=struct.pack("<II",0x20007800,0x08010141)+bytes(0x200-8)
            info={"format":1,"target":"STM32WB35CE","profile":"application","address":"0x08010000",
                  "size":len(data),"crc32":f"{zlib.crc32(data):08x}","sha256":hashlib.sha256(data).hexdigest(),"version":"0.2.0"}
            binary.write_bytes(data);manifest.write_text(json.dumps(info))
            self.assertEqual(package.validate(binary,manifest)[0],data)
            binary.write_bytes(data[:-1]+b"X")
            with self.assertRaises(ValueError):package.validate(binary,manifest)
            binary.write_bytes(data);info["address"]="0x08000000";manifest.write_text(json.dumps(info))
            with self.assertRaises(ValueError):package.validate(binary,manifest)
    def test_hex_records(self):
        data=bytes(range(128))
        result=package.install_hex([(0x08010000,data)])
        rebuilt=bytearray();upper=None
        for line in result.splitlines():
            raw=bytes.fromhex(line[1:]);self.assertEqual(sum(raw)&255,0)
            if raw[3]==4:upper=int.from_bytes(raw[4:-1],"big")
            elif raw[3]==0:
                self.assertEqual(upper,0x0801);rebuilt.extend(raw[4:-1])
        self.assertEqual(rebuilt,data)
if __name__=="__main__":unittest.main()
