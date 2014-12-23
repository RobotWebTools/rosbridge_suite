import backports
import backports.ssl_match_hostname

def find_tornado():
    '''
    Do some magic to find and import our internal copy of the tornado module.

    For the reasons this is necessary see:
    https://github.com/RobotWebTools/rosbridge_suite/issues/154
    https://github.com/RobotWebTools/rosbridge_suite/issues/149
    '''
    import sys
    import os.path
    import rospkg
    rpkg = rospkg.RosPack()
    sys.path = [
        os.path.split(__file__)[0],
        os.path.join(rpkg.get_path('rosbridge_tools'), 'src/rosbridge_tools'),
        ] + sys.path
    import tornado
    import tornado.platform
    import tornado.ioloop
    import tornado.web
    import tornado.websocket
    sys.path = sys.path[2:]
    assert(tornado.version == '4.0.2')
    return tornado

