#!/usr/bin/env python
import roslib
import rospy
import string
import copy

import registry

# Keep track of atomic types and special types
atomics = ['bool', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string']
specials = ['time', 'duration']
    

def getTypeDef(type):
    """ A typedef is a dict containing the following fields:
         - string type
         - string[] fieldnames
         - string[] fieldtypes
         - int[] fieldarraylen
         - string[] examples 
    getTypeDef will return a typedef dict for the specified message type """
    if type in atomics:
        # Atomics don't get a typedef
        return None 
    
    if type in specials:
        # Specials get their type def mocked up
        return _getSpecialTypeDef(type)
    
    # Fetch an instance and return its typedef
    instance = registry.getMessageInstance(type)    
    return _getTypeDef(instance)

def getServiceRequestTypeDef(servicetype):
    """ Returns a typedef dict for the service request class for the specified service type """
    # Get an instance of the service request class and return its typedef
    instance = registry.getServiceRequestInstance(servicetype)
    return _getTypeDef(instance)

def getServiceResponseTypeDef(servicetype):
    """ Returns a typedef dict for the service response class for the specified service type """
    # Get an instance of the service response class and return its typedef 
    instance = registry.getServiceResponseInstance(servicetype)
    return _getTypeDef(instance)

def getTypeDefRecursive(type):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Just go straight into the recursive method
    return _getTypeDefsRecursive(type, [])

def getServiceRequestTypeDefRecursive(servicetype):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Get an instance of the service request class and get its typedef
    instance = registry.getServiceRequestInstance(servicetype)
    typedef = _getTypeDef(instance)
    
    # Return the list of sub-typedefs
    return _getSubTypeDefsRecursive(typedef, [])

def getServiceResponseTypeDefRecursive(servicetype):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Get an instance of the service response class and get its typedef
    instance = registry.getServiceResponseInstance(servicetype)
    typedef = _getTypeDef(instance)
    
    # Return the list of sub-typedefs
    return _getSubTypeDefsRecursive(typedef, [])

def _getTypeDef(instance):
    """ Gets a typedef dict for the specified instance """
    if instance is None or not hasattr(instance, "__slots__") or not hasattr(instance, "_slot_types"):
        return None
    
    fieldnames = []
    fieldtypes = []
    fieldarraylen = []
    examples = []
    for i in xrange(len(instance.__slots__)):
        # Pull out the name
        name = instance.__slots__[i]        
        fieldnames.append(name)
        
        # Pull out the type and determine whether it's an array
        field_type = instance._slot_types[i]
        arraylen = -1
        if field_type[-1:]==']':
            if field_type[-2:-1]=='[':
                arraylen = 0
                field_type = field_type[:-2]
            else:
                split = string.find(field_type, '[')
                arraylen = int(field_type[split+1:-1])
                field_type = field_type[:split]
        fieldarraylen.append(arraylen)
        
        # Get the fully qualified type
        field_instance = getattr(instance, name)
        fieldtypes.append(_typeName(field_type, field_instance))
        
        # Set the example as appropriate
        example = field_instance
        if arraylen>=0:
            example = []
        elif field_type not in atomics:
            example = {}
        examples.append(str(example))
    
    typedef = {
       "type": _typeNameFromInstance(instance),
       "fieldnames": fieldnames,
       "fieldtypes": fieldtypes,
       "fieldarraylen": fieldarraylen,
       "examples": examples
    }
    
    return typedef
    
def _getSpecialTypeDef(type):
    example = None
    if type=="time" or type=="duration":
        example = {
            "type": type,
            "fieldnames": ["sec", "nsec"],
            "fieldtypes": ["int32", "int32"],
            "fieldarraylen": [-1, -1],
            "examples": [ "0", "0" ]
        }
    return example

def _getTypeDefsRecursive(type, typesseen):
    """ returns the type def for this type as well as the type defs for any fields within the type """
    if type in typesseen:
        # Don't put a type if it's already been seen
        return []
    
    # Note that we have now seen this type
    typesseen.append(type)
    
    # Get the typedef for this type and make sure it's not None
    typedef = getTypeDef(type)
    
    return _getSubTypeDefsRecursive(typedef, typesseen)
    
def _getSubTypeDefsRecursive(typedef, typesseen):
    if typedef is None:
        return []
    
    # Create the list of subtypes and get the typedefs for fields
    typedefs = [ typedef ]
    for fieldtype in typedef["fieldtypes"]:
        typedefs = typedefs + _getTypeDefsRecursive(fieldtype, typesseen)
        
    return typedefs

def _typeName(type, instance):
    """ given a short type, and an object instance of that type, 
    determines and returns the fully qualified type """    
    # The fully qualified type of atomic and special types is just their original name
    if type in atomics or type in specials:
        return type
            
    # If the instance is a list, then we can get no more information from the instance.
    # However, luckily, the 'type' field for list types is usually already inflated to the full type.
    if isinstance(instance, list):
        return type
    
    # Otherwise, the type will come from the module and class name of the instance                
    return _typeNameFromInstance(instance)
    
def _typeNameFromInstance(instance):
    mod = instance.__module__
    type = mod[0:string.find(mod, '.')]+"/"+instance.__class__.__name__
    return type   
    