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

        If the input topics_glob is empty, it returns empty lists and everything
        is allowed.

        If the input topics_glob is not empty, it will ensure that the output
        topics_globs contain at least one element (an empty string) that will
        ensure that capabilities don't assume everything is allowed because the
        topics_glob list was empty.
    """
    incoming = set()
    outgoing = set()
    for t in topics_glob:
        if t.startswith(">"):
            # Allow to websocket only, not publishing from the websocket.
            topic_path = t[1:]
            outgoing.add(topic_path)
        elif t.startswith("<"):
            # Allow publishing from the websocket only, not subscribing.
            topic_path = t[1:]
            incoming.add(topic_path)
        else:
            # Allow both subscription and advertise / subscribe.
            incoming.add(t)
            outgoing.add(t)

    # The topics_globs assume that empty lists means allow anything. So we
    # need to insert a dummy value to prevent the list from being empty
    # while at the same time preserving backwards compatilibity in case
    # the input topics_glob is really empty and we want to allow everything.
    if len(topics_glob) == 0:
        return {"incoming": [], "outgoing": [], "bidirectional": [], "all": []}

    # Some topic was specified, in this case none of the returns may be empty lists.
    # we insert `""`, this can only ever match the empty string itself, which is an
    # invalid topic name.
    return {"incoming": list(incoming | set([""])),
            "outgoing": list(outgoing | set([""])),
            "bidirectional": list((incoming & outgoing) | set([""])),
            "all": list(incoming | outgoing | set([""]))}
