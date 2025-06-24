# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from base64 import standard_b64decode, standard_b64encode
from io import BytesIO
from math import ceil, floor, sqrt

from PIL import Image


def encode(string):
    """PNG-compress the string in a square RGB image padded with '\n', return the b64 encoded bytes"""
    string_bytes = string.encode("utf-8")
    length = len(string_bytes)
    width = floor(sqrt(length / 3.0))
    height = ceil((length / 3.0) / width)
    bytes_needed = int(width * height * 3)
    string_padded = string_bytes + (b"\n" * (bytes_needed - length))
    i = Image.frombytes("RGB", (int(width), int(height)), string_padded)
    buff = BytesIO()
    i.save(buff, "png")
    encoded = standard_b64encode(buff.getvalue())
    return encoded.decode()


def decode(string):
    """b64 decode the string, then PNG-decompress and remove the '\n' padding"""
    decoded = standard_b64decode(string)
    buff = BytesIO(decoded)
    i = Image.open(buff, formats=("png",)).convert("RGB")
    dec_str = i.tobytes().decode("utf-8")
    dec_str = dec_str.replace("\n", "")  # Remove padding from encoding
    return dec_str
