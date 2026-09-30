"""Generate the small Latin-1 extension from the existing 7x10 font and own pixels.

No display coordinates, ASCII pixels or external font files are changed.
"""
import argparse
from pathlib import Path
import re
import unicodedata

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "MDK-ARM/RTE/ssd1306_font7x10_extended.h"


def glyphs():
    source = (ROOT / "MDK-ARM/ssd1306_fonts.c").read_text()
    body = source.split("Font7x10 [] = {", 1)[1].split("};", 1)[0]
    rows = [int(s, 16) for s in re.findall(r"0x[0-9a-fA-F]+", body)]
    assert len(rows) == 95 * 10
    ascii_font = {chr(cp): rows[(cp-32)*10:(cp-31)*10] for cp in range(32, 127)}
    result = {}

    def draw(cp, *lines):
        assert len(lines) <= 10 and all(len(line) <= 7 for line in lines)
        result[cp] = [int(line.ljust(7, ".").replace(".", "0").replace("#", "1"), 2) << 9
                      for line in lines] + [0] * (10-len(lines))

    draw(0x80, "..###", ".#...#", ".#", "####", ".#", "####", ".#...#", "..###")  # euro
    draw(0x81, ".#####", ".#...#", ".#...#", ".#...#", ".#...#", ".#...#", ".#####")  # missing
    draw(0xA1, "..#", "", "..#", "..#", "..#", "..#", "..#", "..#")
    draw(0xA2, "...#", "..###", ".#.#.#", ".#.#", ".#.#", ".#.#.#", "..###", "...#")
    draw(0xA3, "..###", ".#...#", ".#", ".#", "####", ".#", ".#", "######")
    draw(0xA4, ".#...#", "..###", "..#.#", "..###", ".#...#")
    draw(0xA5, ".#...#", ".#...#", "..#.#", "...#", ".#####", "...#", ".#####", "...#")
    draw(0xA6, "..#", "..#", "..#", "", "", "..#", "..#", "..#")
    draw(0xA7, "..###", ".#", "..##", ".#..#", ".#..#", "..##", "....#", ".###")
    draw(0xA8, ".#.#")
    draw(0xA9, "..###", ".#...#", "#..##.#", "#.#...#", "#.#...#", "#..##.#", ".#...#", "..###")
    draw(0xAA, "..##", "...#", "..##", ".###", "", ".###")
    draw(0xAB, "", "", "..#..#", ".#..#", "#..#", ".#..#", "..#..#")
    draw(0xAC, "", "", ".#####", ".....#", ".....#")
    result[0xAD] = ascii_font["-"]
    draw(0xAE, "..###", ".#...#", "#.##..#", "#.#.#.#", "#.##..#", "#.#.#.#", ".#...#", "..###")
    draw(0xAF, ".#####")
    draw(0xB0, "..##", ".#..#", ".#..#", "..##")
    draw(0xB1, "...#", "...#", ".#####", "...#", "...#", "", ".#####")
    draw(0xB2, ".##", "...#", "..#", ".###")
    draw(0xB3, ".##", "...#", "..#", "...#", ".##")
    draw(0xB4, "...#", "..#")
    draw(0xB5, "", "", ".#..#", ".#..#", ".#..#", ".##.#", ".#.#.#", ".#", ".#")
    draw(0xB6, ".#####", "#.#.#", "#.#.#", ".##.#", "..#.#", "..#.#", "..#.#", "..#.#")
    draw(0xB7, "", "", "", "..##", "..##")
    draw(0xB8, "", "", "", "", "", "", "", "", "...#", "..#")
    draw(0xB9, "..#", ".##", "..#", ".###")
    draw(0xBA, "..##", ".#..#", ".#..#", "..##", "", ".####")
    draw(0xBB, "", "", "#..#", ".#..#", "..#..#", ".#..#", "#..#")
    draw(0xBC, "#...#", "#..#", "..#", ".#", "#..#.#", "...###", ".....#")
    draw(0xBD, "#...#", "#..#", "..#", ".#", "#..##", ".....#", "...##", "...###")
    draw(0xBE, "##..#", ".#.#", "###", ".#", "#..#.#", "...###", ".....#")
    draw(0xBF, "..#", "", "..#", "..#", ".#", "#", "#...#", ".###")
    draw(0xC6, ".#####", "#..#", "#..#", "#..###", "####", "#..#", "#..#", "#..###")
    draw(0xD0, ".###", ".#..#", ".#...#", "####.#", ".#...#", ".#...#", ".#..#", ".###")
    draw(0xD7, "", "", ".#...#", "..#.#", "...#", "..#.#", ".#...#")
    draw(0xD8, "..###.#", ".#...#", ".#..##", ".#.#.#", ".##..#", ".#...#", "#.#..#", "..###")
    draw(0xDE, ".#", ".#", ".####", ".#...#", ".#...#", ".####", ".#", ".#")
    draw(0xDF, "..##", ".#..#", ".#..#", ".#.#", ".#..#", ".#...#", ".#...#", ".#.##", ".#")
    draw(0xE6, "", "", ".##.##", "...#..#", ".######", "#..#", "#..#..#", ".##.##")
    draw(0xF0, "..#.#", "...#", "..#.#", "....#", "..###", ".#..#", ".#..#", "..##")
    draw(0xF7, "", "...#", "", ".#####", "", "...#")
    draw(0xF8, "", "", "..###.#", ".#..##", ".#.#.#", ".##..#", "#.#..#", "..###")
    draw(0xFE, ".#", ".#", ".####", ".#...#", ".#...#", ".####", ".#", ".#", ".#")
    accent_rows = {0x300: [0x2000, 0x1000], 0x301: [0x0800, 0x1000],
                   0x302: [0x1000, 0x2800], 0x303: [0x2800, 0x5000],
                   0x308: [0x2800, 0], 0x30A: [0x1000, 0x2800]}
    for cp in range(0xC0, 0x100):
        if cp in result:
            continue
        parts = [int(s, 16) for s in unicodedata.decomposition(chr(cp)).split()]
        assert len(parts) == 2 and chr(parts[0]) in ascii_font, hex(cp)
        base, mark = chr(parts[0]), parts[1]
        pixels = list(ascii_font[base])
        if mark == 0x327:  # cedilla below C/c; stays within the same ten rows
            pixels[8:10] = [0x1000, 0x2000]
        else:
            if base.isupper():
                pixels = accent_rows[mark] + pixels[:8]
            else:
                pixels[:2] = accent_rows[mark]
        result[cp] = pixels
    assert set(result) == {0x80, 0x81} | set(range(0xA1, 0x100))
    assert all(len(v) == 10 and all(x & 0x01FF == 0 for x in v) for v in result.values())
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    pixels = glyphs()
    output = "/* Generated by tools/font7x10.py. Fixed 7x10 cells; ASCII stays unchanged. */\n"
    output += "static const uint16_t Font7x10Extended[][10] = {\n"
    for cp in sorted(pixels):
        label = f"U+{0x20AC if cp == 0x80 else 0x25A1 if cp == 0x81 else cp:04X}"
        output += "    {" + ", ".join(f"0x{v:04X}" for v in pixels[cp]) + "}, /* " + label + " */\n"
    output += "};\n"
    if args.check:
        if TARGET.read_text() != output:
            raise SystemExit("Extended font is stale; run python tools/font7x10.py")
        print("Extended 7x10 font: all 97 glyphs match the generator.")
    else:
        TARGET.write_text(output, encoding="ascii")


if __name__ == "__main__":
    main()
