#!/usr/bin/env python
# $URL$
# $Rev$
# Make ICC Profile

# References
#
# [ICC 2001] ICC Specification ICC.1:2001-04 (Profile version 2.4.0)
# [ICC 2004] ICC Specification ICC.1:2004-10 (Profile version 4.2.0.0)

import struct

# Local module.
import iccp

def black(m):
    """Return a function that maps all values from [0.0,m] to 0, and maps
    the range [m,1.0] into [0.0, 1.0] linearly.
    """

    m = float(m)

    def f(x):
        if x <= m:
            return 0.0
        return (x-m)/(1.0-m)
    return f

# For monochrome input the required tags are (See [ICC 2001] 6.3.1.1):
# profileDescription [ICC 2001] 6.4.32
# grayTRC [ICC 2001] 6.4.19
# mediaWhitePoint [ICC 2001] 6.4.25
# copyright [ICC 2001] 6.4.13

def agreyprofile(out):
    it = iccp.Profile().greyInput()
    it.addTags(kTRC=black(0.07))
    it.write(out)

def main():
    import sys
    agreyprofile(sys.stdout)

if __name__ == '__main__':
    main()
