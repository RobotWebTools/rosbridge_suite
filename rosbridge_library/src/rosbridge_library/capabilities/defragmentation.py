from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, ClassVar

from rosbridge_library.capability import Capability

if TYPE_CHECKING:
    from collections.abc import Sequence

    from rosbridge_library.protocol import Protocol


class Defragment(Capability):
    # Dictionary of defragmentation instances
    # Format:
    # {
    #   <<message1_ID>> : {
    #     "timestamp_last_append" : <<datetime-object>>,
    #     "total" : <<total_fragments>>,
    #     "fragment_list" : {
    #       <<fragment1ID>>: <<fragment1_data>>,
    #       <<fragment2ID>>: <<fragment2_data>>,
    #       ...
    #     }
    # }
    lists: ClassVar[dict[str, dict]] = {}

    fragment_timeout = 600
    opcode = "fragment"

    def __init__(self, protocol: Protocol) -> None:
        Capability.__init__(self, protocol)

        # populate parameters
        if self.protocol.parameters is not None:
            self.fragment_timeout = self.protocol.parameters["fragment_timeout"]

        protocol.register_operation(self.opcode, self.defragment)

        self.received_fragments = Defragment.lists

    # defragment() does:
    #   1) take any incoming message with op-code "fragment"
    #   2) check all existing fragment lists for time out
    #       (could be done by a thread but should be okay this way)
    #   2.a) remove timed out lists (only if new fragment is not for this list)
    #       (checking whenever a new fragment is received should suffice)
    #   3) create a new fragment list for new message ids
    #       (to have control over growth of fragment lists)
    #   3.a) check message fields
    #   3.b) append the new fragment to 'the' list
    #   3.c) add time stamp (last_fragment_appended) to 'this' list
    #   4) check if the list of current fragment (message id) is complete
    #   4.a) reconstruct the original message by concatenating the fragments
    #   4.b) pass the reconstructed message string to protocol.incoming()
    #       (protocol.incoming is checking message fields by itself, so no need to do this before
    #       passing the reconstructed message to protocol)
    #   4.c) remove the fragment list to free up memory
    def defragment(self, message: dict[str, Any]) -> None:
        now = time.monotonic()

        msg_id: str | None = message.get("id")
        if msg_id is None:
            self.protocol.log("error", "received invalid fragment! No message ID found.")
            return

        for frag_id in self.received_fragments:
            time_diff = now - self.received_fragments[frag_id]["timestamp_last_append"]
            if (
                time_diff > self.fragment_timeout
                and not self.received_fragments[frag_id]["is_reconstructing"]
            ):
                log_msg = [f"fragment list {frag_id} timed out.."]

                if msg_id != frag_id:
                    log_msg.append(" -> removing it..")
                    del self.received_fragments[frag_id]
                else:
                    log_msg.append(
                        f" -> but we're just about to add fragment #{message.get('num')}"
                        f" of {self.received_fragments[msg_id]['total']} ..keeping the list"
                    )
                self.protocol.log("warning", "".join(log_msg))

        msg_opcode: str | None = message.get("op")
        msg_num: int | None = message.get("num")
        msg_total: int | None = message.get("total")
        msg_data: Sequence | None = message.get("data")

        # Abort if any message field is missing
        if msg_opcode is None or msg_num is None or msg_total is None or msg_data is None:
            self.protocol.log("error", "received invalid fragment!")
            return

        self.protocol.log("debug", "fragment for messageID: " + str(msg_id) + " received.")

        # Create fragment container if none exists yet
        if msg_id not in self.received_fragments:
            self.received_fragments[msg_id] = {
                "is_reconstructing": False,
                "total": msg_total,
                "timestamp_last_append": now,
                "fragment_list": {},
            }
            self.protocol.log("debug", "opened new fragment list for messageID " + str(msg_id))

        # print "received fragments:", len(self.received_fragments[msg_id]["fragment_list"].keys())

        # Add fragment to fragment container's list if not already in list
        if (
            msg_num not in self.received_fragments[msg_id]["fragment_list"]
            and msg_num <= self.received_fragments[msg_id]["total"]
            and msg_total == self.received_fragments[msg_id]["total"]
        ):
            self.received_fragments[msg_id]["fragment_list"][msg_num] = msg_data
            self.received_fragments[msg_id]["timestamp_last_append"] = now

            self.protocol.log(
                "debug",
                f"appended fragment #{msg_num} (total: {msg_total}) to fragment list for messageID {msg_id}",
            )
        else:
            self.protocol.log("error", "error while trying to append fragment " + str(msg_num))
            return

        received_all_fragments = False
        existing_fragments = len(self.received_fragments[msg_id]["fragment_list"])
        announced_total = self.received_fragments[msg_id]["total"]

        # Make sure total number of fragments received
        if existing_fragments == announced_total:
            self.protocol.log(
                "debug",
                f"enough/all fragments for messageID {msg_id} received [ {existing_fragments} ]",
            )
            # Check each fragment matches up
            received_all_fragments = True
            for i in range(announced_total):
                if i not in self.received_fragments[msg_id]["fragment_list"]:
                    received_all_fragments = False
                    self.protocol.log("error", f"fragment #{i} for messageID {msg_id} is missing!")

        self.received_fragments[msg_id]["is_reconstructing"] = received_all_fragments

        if received_all_fragments:
            self.protocol.log("debug", f"reconstructing original message {msg_id}")

            # Reconstruct the message
            reconstructed_msg = "".join(self.received_fragments[msg_id]["fragment_list"].values())

            self.protocol.log("debug", f"reconstructed original message:\n{reconstructed_msg}")

            duration = time.monotonic() - now

            # Pass the reconstructed message to rosbridge
            self.protocol.incoming(reconstructed_msg)

            self.protocol.log(
                "info",
                f"reconstructed message (ID: {msg_id}) from {msg_total} fragments. [duration: {duration} s]",
            )

            # Remove fragmentation container
            del self.received_fragments[msg_id]
            self.protocol.log("debug", f"removed fragment list for messageID {msg_id}")

    def finish(self) -> None:
        self.protocol.unregister_operation("fragment")
