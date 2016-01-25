from rosbridge_library.capability import Capability
from datetime import datetime
import threading

class ReceivedFragments():
    """
    Singleton class to hold lists of received fragments in one 'global' object
    """
    class __impl:
        """ Implementation of the singleton interface """
        def spam(self):
            """ Test method, return singleton id """
            return id(self)

    __instance = None
    # List of defragmentation instances
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
    # },
    # ...
    lists = {}

    def __init__(self):
        """ Create singleton instance """
        if ReceivedFragments.__instance is None:
            ReceivedFragments.__instance = ReceivedFragments.__impl()
            self.lists = {}

        self.__dict__['_ReceivedFragments__instance'] = ReceivedFragments.__instance

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)

class Defragment(Capability, threading.Thread):

    fragment_timeout = 600
    opcode = "fragment"
    global received_fragments

    protocol = None

    def __init__(self, protocol):
        Capability.__init__(self, protocol)

        self.protocol = protocol

        # populate parameters
        if self.protocol.parameters != None:
            self.fragment_timeout = self.protocol.parameters["fragment_timeout"]

        protocol.register_operation(self.opcode, self.defragment)

        self.received_fragments = ReceivedFragments().lists
        threading.Thread.__init__(self)


    # defragment() does:
    #   1) take any incoming message with op-code "fragment"
    #   2) check all existing fragment lists for time out                       # could be done by a thread but should be okay this way:
    #   2.a) remove timed out lists (only if new fragment is not for this list) #   - checking whenever a new fragment is received should suffice
    #   3) create a new fragment list for new message ids                       #     to have control over growth of fragment lists
    #   3.a) check message fields
    #   3.b) append the new fragment to 'the' list
    #   3.c) add time stamp (last_fragment_appended) to 'this' list
    #   4) check if the list of current fragment (message id) is complete
    #   4.a) reconstruct the original message by concatenating the fragments
    #   4.b) pass the reconstructed message string to protocol.incoming()       # protocol.incoming is checking message fields by itself, so no need to do this before passing the reconstructed message to protocol
    #   4.c) remove the fragment list to free up memory                        
    def defragment(self, message):
        now = datetime.now()

        if self.received_fragments != None:
            for id in self.received_fragments.keys() :
                time_diff = now - self.received_fragments[id]["timestamp_last_append"]
                if (time_diff.total_seconds() > self.fragment_timeout and
                    not self.received_fragments[id]["is_reconstructing"]):
                    log_msg = ["fragment list ", str(id), " timed out.."]

                    if message["id"] != id:
                        log_msg.append(" -> removing it..")
                        del self.received_fragments[id]
                    else:
                        log_msg.extend([" -> but we're just about to add fragment #"])
                        log_msg.extend([str(message.get("num")), " of "])
                        log_msg.extend([str(self.received_fragments[message.get("id")]["total"])])
                        log_msg.extend([" ..keeping the list"])
                    self.protocol.log("warning", ''.join(log_msg))

        msg_opcode = message.get("op")
        msg_id = message.get("id")
        msg_num = message.get("num")
        msg_total = message.get("total")
        msg_data = message.get("data")

        # Abort if any message field is missing
        if ((msg_opcode == None) or (msg_id == None) or
            (msg_num == None) or (msg_total == None) or
            (msg_data == None)):
            self.protocol.log("error", "received invalid fragment!")
            return

        log_msg = "fragment for messageID: " + str(msg_id) + " received."
        self.protocol.log("debug", log_msg)

        # Create fragment container if none exists yet
        if msg_id not in self.received_fragments.keys():
            self.received_fragments[msg_id] = {
                "is_reconstructing": False,
                "total": message["total"],
                "timestamp_last_append": now,
                "fragment_list": {}
            }
            log_msg = "opened new fragment list for messageID " + str(msg_id)
            self.protocol.log("debug", log_msg)

        #print "received fragments:", len(self.received_fragments[msg_id]["fragment_list"].keys())

        # Add fragment to fragment container's list if not already in list
        if ((msg_num not in self.received_fragments[msg_id]["fragment_list"].keys() ) and
             msg_num <= self.received_fragments[msg_id]["total"] and
             msg_total == self.received_fragments[msg_id]["total"]
            ):
            self.received_fragments[msg_id]["fragment_list"][msg_num] = msg_data
            self.received_fragments[msg_id]["timestamp_last_append"] = now
            log_msg = ["appended fragment #" + str(msg_num)]
            log_msg.extend([" (total: ", str(msg_total), ") to fragment list for messageID ", str(msg_id)])
            self.protocol.log("debug", ''.join(log_msg))
        else:
            log_msg = "error while trying to append fragment " + str(msg_num)
            self.protocol.log("error", log_msg)
            return

        received_all_fragments = False
        existing_fragments = len(self.received_fragments[msg_id]["fragment_list"])
        announced_total = self.received_fragments[msg_id]["total"]

        # Make sure total number of fragments received
        if existing_fragments == announced_total:
            log_msg = ["enough/all fragments for messageID " + str(msg_id) + " received"]
            log_msg.extend([" [", str(existing_fragments), "]"])
            log_msg = ''.join(log_msg)
            self.protocol.log("debug", log_msg)
            # Check each fragment matches up
            received_all_fragments = True
            for i in range(0, announced_total):
                if i not in self.received_fragments[msg_id]["fragment_list"]:
                    received_all_fragments = False
                    log_msg = "fragment #" +str(i)
                    log_msg += " for messageID " + str(msg_id) + " is missing! "
                    self.protocol.log("error", log_msg)

        self.received_fragments[msg_id]["is_reconstructing"] = received_all_fragments

        if received_all_fragments:
            log_msg = "reconstructing original message " + str(msg_id)
            self.protocol.log("debug", log_msg)

            # Reconstruct the message
            reconstructed_msg = ''.join(self.received_fragments[msg_id]["fragment_list"][0:message["total"]])
            log_msg = ["reconstructed original message:\n"]
            log_msg.append(reconstructed_msg)
            log_msg = ''.join(log_msg)
            self.protocol.log("debug", log_msg)

            duration = datetime.now() - now

            # Pass the reconstructed message to rosbridge
            self.protocol.incoming(reconstructed_msg)
            log_msg = ["reconstructed message (ID:" + str(msg_id) + ") from "]
            log_msg.extend([str(msg_total), " fragments. "])
            # cannot access msg.data if message is a service_response or else!
            #log_msg += "[message length: " + str(len(str(json.loads(reconstructed_msg)["msg"]["data"]))) +"]"
            log_msg.extend(["[duration: ", str(duration.total_seconds()),  " s]"])
            log_msg = ''.join(log_msg)
            self.protocol.log("info", log_msg)

            # Remove fragmentation container
            del self.received_fragments[msg_id]
            log_msg = "removed fragment list for messageID " + str(msg_id)
            self.protocol.log("debug", log_msg)

    def finish(self):
        self.received_fragments = None
        self.protocol.unregister_operation("fragment")
