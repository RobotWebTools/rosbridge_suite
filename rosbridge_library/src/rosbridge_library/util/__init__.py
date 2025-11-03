# try to import json-lib: 1st try ujson, 2nd try simplejson, else import standard Python json
try:
    import ujson as json  # type: ignore[import-untyped]
except ImportError:
    try:
        import simplejson as json  # type: ignore[import-untyped, no-redef]
    except ImportError:
        import json  # type: ignore[no-redef] # noqa: F401

import bson

try:
    _ = bson.BSON
except AttributeError as exc:
    msg = (
        "BSON installation does not support all necessary features. "
        "Please use the MongoDB BSON implementation. "
        "See: https://github.com/RobotWebTools/rosbridge_suite/issues/198"
    )
    raise Exception(msg) from exc
