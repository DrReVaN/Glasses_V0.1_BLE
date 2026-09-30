"""Smartglasses CPU1 OTA client. Requires a Bluetooth adapter and physical approval."""
import argparse
import asyncio
from pathlib import Path
import struct
from package import validate
BASE="-8e22-4541-9d4c-21edae82ed19"
FW="00000011"+BASE
CONTROL="00000023"+BASE
BEGIN="0000fe21"+BASE
DATA="0000fe22"+BASE
END="0000fe23"+BASE
async def update(args,image,info):
    from bleak import BleakClient, BleakScanner
    device=await BleakScanner.find_device_by_address(args.address,timeout=15)
    if device is None: raise RuntimeError("Device not found; disconnect the Android app and wake the glasses.")
    async with BleakClient(device,pair=True,timeout=60,winrt={"use_cached_services":False}) as client:
        version=await client.read_gatt_char(FW)
        if len(version) not in (4,20) or version[:3]!=bytes([0,2,0]): raise RuntimeError("Unsupported BLE protocol")
        if len(version)==20:
            if version[10:12]!=bytes([1,0]): raise RuntimeError("Unsupported OTA protocol")
            print("Installed firmware: " + '.'.join(map(str,struct.unpack('<HHH',version[4:10]))))
        if args.enter:
            if version[3]==1: print("Already in bootloader mode."); return
            await client.write_gatt_char(CONTROL,b"OTA1",response=True)
            print("Approve OTA on the glasses: hold pad 1 for one second (pad 3 cancels). Then run this tool without --enter.")
            return
        if version[3]!=1: raise RuntimeError("Enter the bootloader first using --enter, or hold pad 2 during MCU reset.")
        await asyncio.wait_for(client.write_gatt_char(BEGIN,b"SGU1"+struct.pack("<II",len(image),int(info["crc32"],16)),response=True),15)
        for offset in range(0,len(image),16):
            await asyncio.wait_for(client.write_gatt_char(DATA,struct.pack("<I",offset)+image[offset:offset+16],response=True),5)
            if offset%2048==0: print(f"{offset*100//len(image)}%",flush=True)
        await asyncio.wait_for(client.write_gatt_char(END,b"END1",response=True),10)
        print("100%: device verified and committed the image; reboot follows.")
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("binary",type=Path)
    parser.add_argument("--manifest",type=Path)
    parser.add_argument("--address",help="Bluetooth address, or the device UUID on macOS")
    parser.add_argument("--enter",action="store_true",help="request physically approved reboot into OTA mode")
    parser.add_argument("--check",action="store_true",help="validate package without accessing Bluetooth")
    args=parser.parse_args()
    image,info=validate(args.binary,args.manifest or args.binary.with_suffix(".json"))
    if args.check:print(f"Valid STM32WB35CE app: {len(image)} bytes, SHA-256 {info['sha256']}");return
    if not args.address:parser.error("--address is required unless --check is used")
    asyncio.run(update(args,image,info))
if __name__=="__main__":main()
