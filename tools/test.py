"""Run the actual C core and OTA receiver on the host; GCC or Clang required."""
import argparse
from pathlib import Path
import shlex
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--cc",default="cc",help="C compiler command, e.g. gcc or 'zig cc'")
    p.add_argument("--sanitize",action="store_true")
    args=p.parse_args()
    out=ROOT/"build/tests";out.mkdir(parents=True,exist_ok=True)
    common=shlex.split(args.cc)+["-std=c99","-Wall","-Wextra","-Werror","-I"+str(ROOT/"tests/stubs"),"-I"+str(ROOT/"Core/Inc")]
    if args.sanitize:common+=["-fsanitize=address,undefined","-fno-sanitize-recover=all"]
    suffix=".exe" if sys.platform=="win32" else ""
    for name in ("core","ota","display"):
        exe=out/("test_"+name+suffix)
        cmd=common+["-I"+str(ROOT/"tests/stubs"),str(ROOT/"Core/Src/glasses_core.c"),str(ROOT/("tests/test_"+name+".c"))]
        if name=="ota":
            cmd+=["-Wno-int-to-pointer-cast","-D_GNU_SOURCE","-DSMARTGLASSES_BOOTLOADER","-DGLASSES_HOST_TEST",str(ROOT/"Core/Src/glasses_ota.c")]
        if name=="display":
            cmd += ["-I"+str(ROOT/"tests/display"), "-I"+str(ROOT/"MDK-ARM/RTE"),
                    str(ROOT/"Core/Src/glasses_display.c"), str(ROOT/"MDK-ARM/ssd1306.c"),
                    str(ROOT/"MDK-ARM/ssd1306_fonts.c"), "-lm"]
        subprocess.run(cmd+["-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
if __name__=="__main__":main()
