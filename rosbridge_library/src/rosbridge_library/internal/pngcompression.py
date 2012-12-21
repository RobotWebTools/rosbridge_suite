import Image
from base64 import standard_b64encode, standard_b64decode
from StringIO import StringIO
from math import floor, ceil, sqrt


def encode(string):
    """ PNG-compress the string in a square RBG image padded with '\n', return the b64 encoded bytes """
    length = len(string)
    width = floor(sqrt(length/3.0))
    height = ceil((length/3.0) / width)
    bytes_needed = int(width * height * 3)
    while length < bytes_needed:
        string += '\n'
        length += 1
    i = Image.fromstring('RGB', (int(width), int(height)), string)
    buff = StringIO()
    i.save(buff, "png")
    encoded = standard_b64encode(buff.getvalue())
    return encoded

def decode(string):
    """ b64 decode the string, then PNG-decompress """
    decoded = standard_b64decode(string)
    buff = StringIO(decoded)
    i = Image.open(buff)
    return i.tostring()
