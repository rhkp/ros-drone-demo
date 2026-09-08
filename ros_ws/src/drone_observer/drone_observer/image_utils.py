import struct
import zlib


def _chunk(kind, payload):
    return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind + payload) & 0xffffffff)


def write_rgb_png(path, width, height, rgb):
    """Write a dependency-free RGB PNG for portable evidence artifacts."""
    rows = b''.join(b'\x00' + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height))
    data = b'\x89PNG\r\n\x1a\n'
    data += _chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
    data += _chunk(b'IDAT', zlib.compress(rows, 6))
    data += _chunk(b'IEND', b'')
    with open(path, 'wb') as stream:
        stream.write(data)


def synthetic_rgb(width, height, color=(40, 100, 55)):
    pixels = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            i = (y * width + x) * 3
            shade = (x * 17 + y * 7) % 24
            pixels[i:i + 3] = bytes(max(0, min(255, c + shade)) for c in color)
    return bytes(pixels)
