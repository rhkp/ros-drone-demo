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


def image_to_rgb(image):
    """Convert common ROS image encodings to tightly packed RGB bytes."""
    encoding = image.encoding.lower()
    channels = {
        'rgb8': ('rgb', 3),
        'bgr8': ('bgr', 3),
        'rgba8': ('rgba', 4),
        'bgra8': ('bgra', 4),
        'mono8': ('mono', 1),
        '8uc1': ('mono', 1),
    }.get(encoding)
    if channels is None:
        raise ValueError(f'Unsupported camera encoding: {image.encoding}')

    order, bytes_per_pixel = channels
    width, height = int(image.width), int(image.height)
    row_bytes = width * bytes_per_pixel
    stride = int(image.step or row_bytes)
    raw = bytes(image.data)
    if len(raw) < stride * height:
        raise ValueError('Camera image data is shorter than its declared dimensions')

    rgb = bytearray(width * height * 3)
    for y in range(height):
        source = raw[y * stride:y * stride + row_bytes]
        destination = y * width * 3
        if order == 'rgb':
            rgb[destination:destination + width * 3] = source
        elif order == 'bgr':
            for index in range(0, row_bytes, 3):
                rgb[destination + index:destination + index + 3] = source[index:index + 3][::-1]
        elif order == 'rgba':
            for index in range(width):
                rgb[destination + index * 3:destination + index * 3 + 3] = source[index * 4:index * 4 + 3]
        elif order == 'bgra':
            for index in range(width):
                pixel = source[index * 4:index * 4 + 3]
                rgb[destination + index * 3:destination + index * 3 + 3] = pixel[::-1]
        else:
            for index, value in enumerate(source):
                offset = destination + index * 3
                rgb[offset:offset + 3] = bytes((value, value, value))
    return bytes(rgb)
