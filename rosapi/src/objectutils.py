#!/usr/bin/env python
from string import find

from rosbridge_library.internal import ros_loader

# Keep track of atomic types and special types
atomics = ['bool', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string']
specials = ['time', 'duration']
    

def get_typedef(type):
    """ A typedef is a dict containing the following fields:
         - string type
         - string[] fieldnames
         - string[] fieldtypes
         - int[] fieldarraylen
         - string[] examples 
    get_typedef will return a typedef dict for the specified message type """
    if type in atomics:
        # Atomics don't get a typedef
        return None 
    
    if type in specials:
        # Specials get their type def mocked up
        return _get_special_typedef(type)
    
    # Fetch an instance and return its typedef
    instance = ros_loader.get_message_instance(type)    
    return _get_typedef(instance)

def get_service_request_typedef(servicetype):
    """ Returns a typedef dict for the service request class for the specified service type """
    # Get an instance of the service request class and return its typedef
    instance = ros_loader.get_service_request_instance(servicetype)
    return _get_typedef(instance)

def get_service_response_typedef(servicetype):
    """ Returns a typedef dict for the service response class for the specified service type """
    # Get an instance of the service response class and return its typedef 
    instance = ros_loader.get_service_response_instance(servicetype)
    return _get_typedef(instance)

def get_typedef_recursive(type):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Just go straight into the recursive method
    return _get_typedefs_recursive(type, [])

def get_service_request_typedef_recursive(servicetype):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Get an instance of the service request class and get its typedef
    instance = ros_loader.get_service_request_instance(servicetype)
    typedef = _get_typedef(instance)
    
    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])

def get_service_response_typedef_recursive(servicetype):
    """ Returns a list of typedef dicts for this type and all contained type fields """
    # Get an instance of the service response class and get its typedef
    instance = ros_loader.get_service_response_instance(servicetype)
    typedef = _get_typedef(instance)
    
    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])

def _get_typedef(instance):
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
                split = find(field_type, '[')
                arraylen = int(field_type[split+1:-1])
                field_type = field_type[:split]
        fieldarraylen.append(arraylen)
        
        # Get the fully qualified type
        field_instance = getattr(instance, name)
        fieldtypes.append(_type_name(field_type, field_instance))
        
        # Set the example as appropriate
        example = field_instance
        if arraylen>=0:
            example = []
        elif field_type not in atomics:
            example = {}
        examples.append(str(example))
    
    typedef = {
       "type": _type_name_from_instance(instance),
       "fieldnames": fieldnames,
       "fieldtypes": fieldtypes,
       "fieldarraylen": fieldarraylen,
       "examples": examples
    }
    
    return typedef
    
def _get_special_typedef(type):
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

def _get_typedefs_recursive(type, typesseen):
    """ returns the type def for this type as well as the type defs for any fields within the type """
    if type in typesseen:
        # Don't put a type if it's already been seen
        return []
    
    # Note that we have now seen this type
    typesseen.append(type)
    
    # Get the typedef for this type and make sure it's not None
    typedef = get_typedef(type)
    
    return _get_subtypedefs_recursive(typedef, typesseen)
    
def _get_subtypedefs_recursive(typedef, typesseen):
    if typedef is None:
        return []
    
    # Create the list of subtypes and get the typedefs for fields
    typedefs = [ typedef ]
    for fieldtype in typedef["fieldtypes"]:
        typedefs = typedefs + _get_typedefs_recursive(fieldtype, typesseen)
        
    return typedefs

def _type_name(type, instance):
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
    return _type_name_from_instance(instance)
    
def _type_name_from_instance(instance):
    mod = instance.__module__
    type = mod[0:find(mod, '.')]+"/"+instance.__class__.__name__
    return type   
    