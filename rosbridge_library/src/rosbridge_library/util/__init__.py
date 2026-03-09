# try to import json-lib: 1st try ujson, 2nd try simplejson, else import standard Python json
try:
    import ujson as json  # type: ignore[import-untyped]
except ImportError:
    try:
        import simplejson as json  # type: ignore[import-untyped, no-redef]
    except ImportError:
        import json  # type: ignore[no-redef] # noqa: F401
