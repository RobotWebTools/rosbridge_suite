### Tornado Version

Often the server breaks because it is using the wrong version of Tornado.  Tornado's interfaces and behavior change very quickly, Rosbridge is only tested against the version installed by rosdep.  On Debian-based systems, this is the apt package python-tornado.

To check your imported Tornado version, run:

```
python -c 'import tornado; print tornado.version'
```

The imported version should the `apt` version:

```
apt-cache show python-tornado | grep Version
```

If the versions don't match, it's likely that the wrong version of Tornado was installed with `pip`.  Try uninstalling it:

```
pip uninstall tornado
```

If the wrong version is still imported, you will need to find it and remove it.  You might find it with something like:

```
find /usr/local/lib/python*/*-packages/ -type d | grep '/tornado/'
```
