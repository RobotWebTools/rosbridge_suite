# try to import json-lib: 1st try usjon, 2nd try simplejson, else import standard python json
try:
    import ujson as json
except ImportError:
    try:
        import simplejson as json
    except ImportError:
        import json
