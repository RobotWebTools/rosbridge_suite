# try to import json-lib: 1st try usjon, 2nd try simplejson, else import standard python json
try:
    import ujson as json
except ImportError:
    try:
        import simplejson as json
    except ImportError:
        import json

# Differing string types for Python 2 and 3
import sys
if sys.version_info >= (3, 0):
    string_types = (str,)
else:
    string_types = (str, unicode)

def appropriate_bson_installed():
    """ Check whether appropriate bson module is installed or not

    pymongo oriented bson module should be installed. See following issue for
    detail.

    * https://github.com/RobotWebTools/rosbridge_suite/issues/198
    """
    import bson
    try:
        bson.BSON
    except AttributeError:
        return False
    return True
