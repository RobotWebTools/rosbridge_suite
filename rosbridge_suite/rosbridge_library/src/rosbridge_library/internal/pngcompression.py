from pypng.code import png
from base64 import standard_b64encode, standard_b64decode
from StringIO import StringIO


def encode(string):
    """ PNG-compress the string, return the b64 encoded bytes """
    bytes = list(bytearray(string))
    png_image = png.from_array([bytes], 'L')
    buff = StringIO()
    png_image.save(buff)
    encoded = standard_b64encode(buff.getvalue())
    return encoded

def decode(string):
    """ b64 decode the string, then PNG-decompress """
    decoded = standard_b64decode(string)
    reader = png.Reader(bytes=decoded)
    width, height, rawpixels, metadata = reader.read()
    pixels = list(rawpixels)[0]
    return str(bytearray(pixels))
