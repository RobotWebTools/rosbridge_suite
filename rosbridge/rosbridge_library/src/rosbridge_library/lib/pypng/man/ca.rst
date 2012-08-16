.. $URL$
.. $Rev$

Why Use PyPNG?
==============

In order to motivate you into using PyPNG this document discusses some
of the reasons for using PyPNG over other software.

For starters I assume you already love PNG.  If you don't want to read
and write PNG files then you don't want to use PyPNG.

The most obvious "competitor" to PyPNG is PIL.  Depending on what job
you want to do you might also want to use Netpbm (PyPNG can convert to
and from the Netpbm PNM format), or use ``ctypes`` to interface directly to a
compiled version of libpng.  If you know of others, let me know.

PyPNG is written in Python.  Python is clearly the coolest sexiest
language going, so PyPNG is cool and sexy too.  By association.

Because PyPNG is written in Python it's trivial to install into a Python
installation, or include in your application (as long as it's written in
Python!).  Just use ``python setup.py install`` or just copy the
``code/png.py`` file.  You can even `curl` it straight into wherever you
need it: ``curl -O http://pypng.googlecode.com/svn/trunk/code/png.py``.
In fact, I have never managed to install PIL and that was one of the
motivating factors for PyPNG.  So PyPNG will always be easy to install.

PyPNG can read and write all PNG formats.  PNG supports a generous
variety of image formats: RGB or greyscale, with or without an alpha
channel; and a choice of bit depths from 1,2, or 4 (as long as you want
greyscale or a pallete), 8, and 16 (but 16 bits is not allowed for
palettes).  A pixel can vary in size from 1 to 64 bits:
1/2/4/8/16/24/32/48/64.  In addition a PNG file can be `interlaced` or
not.  An interlaced file allows an incrementally refined display of
images being downloaded over slow links.

PIL, according to its handbook, does not support interlaced PNG
files.  Also, PIL only has internal representations (PIL `mode`)
for 1-bit and 8-bit channel values.  This makes me wonder if PIL
can read PNG files with bit depth 2 or 4 (greyscale or palette),
and also bit depth 16 (which PNG supports for greyscale and RGB
images).  In addition, PIL has only "limited support" for greyscale
with alpha, which is one of PNG's supported formats (in 8 and 16
bits per channel).

I don't mean to belittle PIL here, PIL's focus is not PNG.  PIL's focus
is image processing, and this is where PyPNG sucks.  If you want to
actually process an image---resize, rotate, composite, crop--then you
should use PIL.  This is simply out of scope for PyPNG.  In PyPNG you
get the image as basically an array of numbers.  So some image
processing is possible fairly easily, for example cropping to integer
coordinates, or gamma conversion, but in any case PyPNG provides no
support for it.  In the future a sister project to PyPNG may add some
simple image processing, but processing in pure Python will be way slow.

PyPNG (when used in its command-line mode) can read and write Netpbm
PAM files.  PAM is useful as an intermediary format for performing
processing; it allows the pixel data to be transferred in a simple format
that is easily processed.  A typical workflow using PAM might be:

PNG to PAM ... process PAM file (for example, resize) ... PAM to PNG

Using PAM as an intermediate format is preferred over having to
carry around your alpha channel in a separate file; it allows
workflows to be pipelined more easily.

When reading PAM files a single source file can be used to create
PNG files with all permitted channel combinations: greyscale,
greyscale--alpha, RGB, RGBA.  When writing a PAM file is created for
those PNG formats with an alpha channel, otherwise a compatible PGM or
PPM file is created.

Netpbm's support for PAM to PNG conversion is more limited than PyPNG's.
Netpbm will only convert a source PAM that has 4 channels (for example it does
not create greyscale--alpha PNG files from ``GRAYSCALE_ALPHA`` PAM files).
Netpbm's usual tool for create PNG files, ``pnmtopng``, requires an alpha
channel to be specified in a separate file.

PyPNG has good support for PNG's ``sBIT`` chunk.  This allows end to end
processing of files with any bit depth from 1 to 16 (for example a
10-bit scanner may use the ``sBIT`` chunk to declare that the samples in
a 16-bit PNG file are rescaled 10-bit samples; in this case, PyPNG
delivers 10-bit samples).  Netpbm handle's the ``sBIT`` chunk in a
similar way, but other toolsets may not (PIL?).

``libpng`` is made by the PNG gods, so if want to get at all that
goodness, then you may want to interface directly to libpng via
``ctypes``.  That could be a good idea for some things.  Installation
would be trickier.
