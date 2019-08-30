import socket


def get_ephemeral_port(sock_family=socket.AF_INET, sock_type=socket.SOCK_STREAM):
    """Return an ostensibly available ephemeral port number."""
    # We expect that the operating system is polite enough to not hand out the
    # same ephemeral port before we can explicitly bind it a second time.
    s = socket.socket(sock_family, sock_type)
    s.bind(('', 0))
    port = s.getsockname()[1]
    s.close()
    return port
