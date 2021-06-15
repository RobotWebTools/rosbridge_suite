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


def directional_topics_globs(topics_glob):
    """
        Return a filtered set of topic globs, specifying which topics are allowed
        in incoming (from websocket to ros), outgoing (from ros to websocket) or
        in both directions.

        By default topic communication is allowed in both directions.
        Topics prefixed with `>` are only allowed in outgoing (to the websocket).
        Topics prefixed with `<` are only allowed in ingoing (from the websocket).
    """
    incoming = set()
    outgoing = set()
    for t in topics_glob:
        if t.startswith(">"):
            # Allow to websocket only, not publishing from the websocket.
            topic_path = t[1:]
            outgoing.add(topic_path)
        elif t.startswith("<"):
            # Allow publishing from the websocket only, not subscribing to the messages.
            topic_path = t[1:]
            incoming.add(topic_path)
        else:
            # Allow both subscription and advertise / subscribe.
            incoming.add(t)
            outgoing.add(t)
    return {"incoming": list(incoming),
            "outgoing": list(outgoing),
            "bidirectional": list(incoming & outgoing),
            "all": list(incoming | outgoing)}
