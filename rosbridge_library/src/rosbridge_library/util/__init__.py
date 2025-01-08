# try to import json-lib: 1st try ujson, 2nd try simplejson, else import standard Python json
try:
    import ujson as json  # type: ignore[import-untyped]
except ImportError:
    try:
        import simplejson as json  # type: ignore[import-untyped]
    except ImportError:
        import json  # noqa: F401

import bson

try:
    bson.BSON
except AttributeError:
    raise Exception(
        "BSON installation does not support all necessary features. "
        "Please use the MongoDB BSON implementation. "
        "See: https://github.com/RobotWebTools/rosbridge_suite/issues/198"
    )
