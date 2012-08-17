#!/usr/bin/env python
# $URL$
# $Rev$
# PNG Histogram
# Only really works on grayscale images.

from array import array
import getopt

import png

def decidemax(level):
    """Given an array of levels, decide the maximum value to use for the
    histogram.  This is normally chosen to be a bit bigger than the 99th
    percentile, but if the 100th percentile is not much more (within a
    factor of 2) then the 100th percentile is chosen.
    """

    truemax = max(level)
    sl = level[:]
    sl.sort(reverse=True)
    i99 = int(round(len(level)*0.01))
    if truemax <= 2*sl[i99]:
        return truemax
    return 1.05*sl[i99]

def hist(out, inp, verbose=None):
    """Open the PNG file `inp` and generate a histogram."""

    r = png.Reader(file=inp)
    x,y,pixels,info = r.asDirect()
    bitdepth = info['bitdepth']
    level = [0]*2**bitdepth
    for row in pixels:
        for v in row:
            level[v] += 1
    maxlevel = decidemax(level)

    h = 100
    outbitdepth = 8
    outmaxval = 2**outbitdepth - 1
    def genrow():
        for y in range(h):
            y = h-y-1
            # :todo: vary typecode according to outbitdepth
            row = array('B', [0]*len(level))
            fl = y*maxlevel/float(h)
            ce = (y+1)*maxlevel/float(h)
            for x in range(len(row)):
                if level[x] <= fl:
                    # Relies on row being initialised to all 0
                    continue
                if level[x] >= ce:
                    row[x] = outmaxval
                    continue
                frac = (level[x] - fl)/(ce - fl)
                row[x] = int(round(outmaxval*frac))
            yield row
    w = png.Writer(len(level), h, gamma=1.0,
      greyscale=True, alpha=False, bitdepth=outbitdepth)
    w.write(out, genrow())
    if verbose: print >>verbose, level

def main(argv=None):
    import sys

    if argv is None:
        argv = sys.argv
    argv = argv[1:]
    opt,arg = getopt.getopt(argv, '')

    if len(arg) < 1:
        f = sys.stdin
    else:
        f = open(arg[0])
    hist(sys.stdout, f)

if __name__ == '__main__':
    main()
