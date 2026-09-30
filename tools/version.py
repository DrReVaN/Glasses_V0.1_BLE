"""Read the release version from one header and validate its binary identity."""
from pathlib import Path
import re
import struct

ROOT = Path(__file__).resolve().parents[1]
OFFSET = 0x140

def parse(text):
    if not isinstance(text, str) or not re.fullmatch(r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)', text):
        raise ValueError('Expected canonical major.minor.patch')
    numbers = tuple(map(int, text.split('.')))
    if any(n > 65535 for n in numbers): raise ValueError('Version components must fit uint16')
    return numbers

def current():
    header = (ROOT / 'Core/Inc/glasses_version.h').read_text()
    numbers = [int(re.search(r'^#define GLASSES_VERSION_' + name + r' ([0-9]+)$', header, re.M)[1])
               for name in ('MAJOR', 'MINOR', 'PATCH')]
    text = '.'.join(map(str, numbers)); parse(text)
    return text

def validate_identity(image, text, mode=0):
    numbers = parse(text)
    if numbers < (0, 3, 0):
        if numbers != (0, 2, 0): raise ValueError('Unsupported legacy firmware')
        if image[OFFSET:OFFSET + 4] == b'SGV1': raise ValueError('Identified image mislabeled as legacy firmware')
        return  # Original 0.2.0 predates the identity field.
    expected = b'SGV1' + struct.pack('<HHHBB', *numbers, 1, mode)
    if image[OFFSET:OFFSET + 12] != expected:
        raise ValueError('Manifest version does not match the embedded binary identity')

if __name__ == '__main__': print(current())
