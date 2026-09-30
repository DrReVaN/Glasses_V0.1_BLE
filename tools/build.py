"""Build both STM32WB35 images with Arm GNU Toolchain (no IDE required)."""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import zlib
from version import current, validate_identity
ROOT = Path(__file__).resolve().parents[1]
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--toolchain-bin", type=Path)
    args = parser.parse_args()
    suffix = ".exe" if sys.platform == "win32" else ""
    def tool(name):
        path = args.toolchain_bin / (name + suffix) if args.toolchain_bin else shutil.which(name)
        if not path or not Path(path).is_file(): raise SystemExit("Install Arm GNU Toolchain and set --toolchain-bin.")
        return str(path)
    gcc, objcopy, size = (tool("arm-none-eabi-" + n) for n in ("gcc", "objcopy", "size"))
    project = ET.parse(ROOT / "MDK-ARM/WB35Test.uvprojx")
    target = project.find(".//Target")
    sources = [ROOT / "MDK-ARM" / x.text.replace("\\", "/") for x in target.findall(".//FilePath") if x.text.endswith(".c")]
    sources = list(dict.fromkeys(p.resolve() for p in sources))
    sources += [ROOT / "Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_iwdg.c"]
    sources = list(dict.fromkeys(sources))
    includes = target.find(".//Cads/VariousControls/IncludePath").text
    include_flags = ["-I" + str((ROOT / "MDK-ARM" / p.strip().replace("\\", "/")).resolve()) for p in includes.split(";") if p.strip()]
    flags = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
             "-Os", "-g3", "-ffunction-sections", "-fdata-sections", "-fno-common",
             "-std=c99", "-Wall", "-Wextra", "-Wno-unused-parameter",
             "-DUSE_HAL_DRIVER", "-DSTM32WB35xx"] + include_flags
    for profile, offset in (("bootloader", 0), ("application", 0x10000)):
        out = ROOT / "build" / profile
        out.mkdir(parents=True, exist_ok=True)
        options = flags + ["-DGLASSES_VECTOR_OFFSET=" + hex(offset)]
        if profile == "bootloader": options += ["-DSMARTGLASSES_BOOTLOADER"]
        def compile_one(path):
            obj = out / (path.stem + ".o")
            command = [gcc] + options + ["-c", str(path), "-o", str(obj)]
            run = subprocess.run(command, capture_output=True, text=True)
            return obj, run
        objects = []
        paths = sources + [ROOT / "GCC/startup_stm32wb35.s", ROOT / "GCC/syscalls.c"]
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(compile_one, paths))
        failed = False
        for obj, result in results:
            objects.append(str(obj))
            if result.stderr: print(result.stderr, end="")
            failed |= result.returncode != 0
        if failed: raise SystemExit(1)
        elf = out / "smartglasses.elf"
        subprocess.run([gcc] + options + objects + [
            "-T" + str(ROOT / ("GCC/" + profile + ".ld")), "--specs=nano.specs", "--specs=nosys.specs",
            "-Wl,--gc-sections", "-Wl,--fatal-warnings", "-Wl,-Map=" + str(out / "smartglasses.map"), "-Wl,--print-memory-usage",
            "-lm", "-o", str(elf)], check=True)
        for fmt, extension in (("binary", "bin"), ("ihex", "hex")):
            subprocess.run([objcopy, "-O", fmt, str(elf), str(out / ("smartglasses." + extension))], check=True)
        subprocess.run([size, str(elf)], check=True)
        binary = (out / "smartglasses.bin").read_bytes()
        manifest = {"format": 1, "target": "STM32WB35CE", "address": hex(0x08000000 + offset),
                    "size": len(binary), "crc32": f"{zlib.crc32(binary):08x}",
                    "sha256": hashlib.sha256(binary).hexdigest(), "version": current(), "profile": profile,
                    "protocol": 1, "min_bootloader": "0.2.0"}
        validate_identity(binary, manifest["version"], 1 if profile == "bootloader" else 0)
        (out / "smartglasses.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("Both images built. See docs/OTA.md before first installation.")
if __name__ == "__main__": main()
