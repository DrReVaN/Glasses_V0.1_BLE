"""Validate app manifest and produce the one-time SWD installation image."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib
def validate(binary, manifest):
    image = binary.read_bytes()
    info = json.loads(manifest.read_text())
    if info.get("format") != 1 or info.get("profile") != "application" or info.get("target") != "STM32WB35CE" or int(info["address"],0) != 0x08010000:
        raise ValueError("Wrong image target, layout or format")
    if not 0x140 <= len(image) <= 0x30000 or info["size"] != len(image):
        raise ValueError("Wrong image size")
    if hashlib.sha256(image).hexdigest() != info["sha256"] or zlib.crc32(image) != int(info["crc32"],16):
        raise ValueError("Image digest mismatch")
    sp, entry = struct.unpack_from("<II", image)
    if sp & 7 or not 0x20000008 < sp <= 0x20008000 or not entry & 1 or not 0x08010000 <= entry & ~1 < 0x08010000 + len(image):
        raise ValueError("Invalid application vector table")
    return image, info
def hex_record(address, kind, data):
    raw = bytes([len(data), address >> 8, address & 255, kind]) + data
    return ":" + (raw + bytes([-sum(raw) & 255])).hex().upper()
def install_hex(regions):
    records=[]; upper=None
    for base, data in sorted(regions):
        for offset in range(0,len(data),16):
            address=base+offset
            if address >> 16 != upper:
                upper=address >> 16;records.append(hex_record(0,4,struct.pack(">H",upper)))
            records.append(hex_record(address & 65535,0,data[offset:offset+16]))
    records.append(hex_record(0,1,b""))
    return "\n".join(records)+"\n"
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--build",type=Path,default=Path(__file__).resolve().parents[1]/"build")
    args=parser.parse_args()
    image,info=validate(args.build/"application/smartglasses.bin",args.build/"application/smartglasses.json")
    loader=(args.build/"bootloader/smartglasses.bin").read_bytes()
    if not 0x140 <= len(loader) <= 0xE000: raise ValueError("Bootloader overlaps device keys")
    sp,entry=struct.unpack_from("<II",loader)
    if not 0x20000008<sp<=0x20008000 or not entry&1 or not 0x08000000<=entry&~1<0x08000000+len(loader):
        raise ValueError("Invalid bootloader vectors")
    metadata=struct.pack("<IIII",len(image),int(info["crc32"],16),0x53475531,1)
    (args.build/"metadata.bin").write_bytes(metadata)
    (args.build/"install.hex").write_text(install_hex([(0x08000000,loader),(0x0800F000,metadata),(0x08010000,image)]))
    print("Created build/install.hex. It contains CPU1 only; it does not modify CPU2/FUS or the keys page.")
if __name__=="__main__":main()
