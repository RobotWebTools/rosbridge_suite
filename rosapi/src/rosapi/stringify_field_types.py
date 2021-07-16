from rosbridge_library.internal import ros_loader

from rosidl_parser.definition import AbstractGenericString, AbstractSequence, AbstractType, AbstractWString, Array, BasicType, NamedType, NamespacedType
from rosidl_adapter.msg import MSG_TYPE_TO_IDL

IDL_TYPE_TO_MSG = {v: k for k, v in MSG_TYPE_TO_IDL.items()}

def stringify_field_types(root_type):
    definition = ""
    seen_types = set()
    deps = [root_type]
    is_root = True
    while deps:
        ty = deps.pop()
        if not is_root:
            definition += "\n================================================================================\n"
            definition += f"MSG: {ty}\n"
        is_root = False

        cls = ros_loader.get_message_class(ty)
        pkg = ty.split("/")[0]
        fields_and_types = zip(cls._fields_and_field_types.keys(), cls.SLOT_TYPES)
        for name, ty in fields_and_types:
            type_str, dep = _stringify_type(ty, pkg)
            if dep is not None and dep not in seen_types:
                deps.append(dep)
                seen_types.add(dep)
            definition += f"{type_str} {name}\n"
    return definition

def _stringify_type(ty: AbstractType, pkg: str):
    if isinstance(ty, BasicType):
        return IDL_TYPE_TO_MSG[ty.typename], None
    elif isinstance(ty, NamedType):
        return ty.name, f"{pkg}/{ty.name}"
    elif isinstance(ty, NamespacedType):
        namespaced_name = ty.namespaced_name()
        if len(namespaced_name) == 3:
            full_name = f"{namespaced_name[0]}/{namespaced_name[2]}"
        elif len(namespaced_name) == 2:
            full_name = f"{namespaced_name[0]}/{namespaced_name[1]}"
        else:
            full_name = "/".join(namespaced_name)
            raise RuntimeError(f"Unsupported namespaced type: f{full_name}")
        is_builtin = ty.namespaces[0] == "builtin_interfaces"
        return full_name, None if is_builtin else full_name
    elif isinstance(ty, AbstractGenericString):
        typename = "wstring" if isinstance(ty, AbstractWString) else "string"
        bound = f"<={ty.maximum_size}" if ty.has_maximum_size() else ""
        return f"{typename}{bound}", None
    elif isinstance(ty, Array):
        name, dep = _stringify_type(ty.value_type, pkg)
        return f"{name}[{ty.size}]", dep
    elif isinstance(ty, AbstractSequence):
        bound = f"[<={ty.maximum_size}]" if ty.has_maximum_size() else "[]"
        name, dep = _stringify_type(ty.value_type, pkg)
        return f"{name}{bound}", dep
    else:
        raise RuntimeError(f"Unhandled type {type(ty).__name__}")
