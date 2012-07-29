#!/usr/bin/env python
import roslib
import rospy

# Variables containing the manifests and modules that have been loaded thus far
loaded_manifests = {}
loaded_modules = {}

def getMessageClass(type):
    """ Loads the message type specified. Returns the loaded class, or None on failure """  
    modname, classname = _splittype(type)
    if modname and classname:
        return _getClass(modname, 'msg', classname)
    else:
        return None

def getServiceClass(type):
    """ Loads the service type specified. Returns the loaded class, or None on failure """
    modname, classname = _splittype(type)
    if modname and classname:
        return _getClass(modname, 'srv', classname)
    else:
        return None

def getMessageInstance(type):
    """ If not loaded, loads the specified type. Then returns an instance of it, or None. """    
    cls = getMessageClass(type)
    if cls:
        return cls()
    else:
        return None

def getServiceInstance(type):
    """ If not loaded, loads the specified type. Then returns an instance of it, or None. """    
    cls = getServiceClass(type)
    if cls:
        return cls()
    else:
        return None    

def getServiceRequestInstance(type):
    cls = getServiceClass(type)
    if cls:
        return cls._request_class()
    else:
        return None

def getServiceResponseInstance(type):
    cls = getServiceClass(type)
    if cls:
        return cls._response_class()
    else:
        return None

def _splittype(type):
    splits = type.split("/")
    while len(splits)>0 and splits[0]=='':
        del splits[0]
    if len(splits)==2:
        return splits
    else:
        return (None, None)

def _getClass(modname, subname, classname):
    """ If not loaded, loads the specified type. Then returns an instance of it, or None. """    
    # Load in the appropriate manifest
    module = _loadClass(modname, subname, classname)
    
    # Create and return an instance
    if module:
        try:
            return getattr(getattr(getattr(module,subname),'_' + classname), classname)
        except:
            print "Unable to load class %s" % (classname,)
    
    # Return None if unable to load
    return None

def _loadClass(modname, subname, classname):
    """ Loads the manifest and imports the module that contains the specified type
    Returns the loaded module, or None on failure """
    global loaded_manifests, loaded_modules
    
    if not (modname in loaded_manifests):
        try:
            roslib.load_manifest(modname)    
            loaded_manifests[modname] = True;
        except:
            print "Unable to load manifest for module %s" % (modname,)
            return None

    fullname = modname + '.'+ subname + '._' + classname
    if not (fullname in loaded_modules):
        try:
            loaded_modules[fullname] = __import__(fullname)
        except:
            print "Unable to import %s" % (fullname,)
            return None

    # Module has been successfully imported
    return loaded_modules[fullname]


    