from rosbridge_library.capability import Capability                             # import superclass
from datetime import datetime                                                   # needed for time stamps
import threading                                                                # needed for creating Threads for instances of Defragmentation class

# import needed for debug lines below.. 
#import sys                                                                     # needed for "getSizeOf.."
#import cPickle                                                                 # needed to serialize the fragment_lists and calculate used memory

#####################################################################################################################################
#TODO:  documentation
#TODO:  type checking
#TODO:  exception handling
# optional ##########################################################################################################################
#TODO:  use String.join() instead of "+=" to improve performance during message reconstruction
#          ..see details below (comment near code for creating a new fragment list..)
#####################################################################################################################################


### "Singleton" class to hold lists of received fragments in one (!) 'global' object ################################################
class ReceivedFragments():
    class __impl:                                                               # stuff to make an object of this class a 'singleton' (or something alike)
        """ Implementation of the singleton interface """                       # ..in the end it's just the instance pointer that is needed.. (?)
        def spam(self):
            """ Test method, return singleton id """
            return id(self)

    __instance = None                                                           # storage for the instance reference
    lists = {}                                                                  # lists will be available to each "client's Defragmentation instance"
                                                                                # ..initialize empty
    def __init__(self):
        """ Create singleton instance """
        if ReceivedFragments.__instance is None:                                # Check whether we already have an instance
            ReceivedFragments.__instance = ReceivedFragments.__impl()           # Create and remember instance
            self.lists = {}

        self.__dict__['_ReceivedFragments__instance'] = ReceivedFragments.__instance    # Store instance reference as the only member in the handle

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)
#####################################################################################################################################


# main functionality is provided by the method 'defragment()'
class Defragment(Capability, threading.Thread):

    # initialization of options
    fragment_timeout = 600 # in seconds # 600 -> 10 minutes                     # timeout after last received fragment to free up memory from unfinished (but timed out) fragment lists
    opcode = "fragment"                                                         # this opcode will be registered in protocol
    global received_fragments                                                   # this will be a dict and hold the fragment lists

    # constructor
    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)                                     # set protocol by calling __init__ of superclass

        # Register the operations that this capability provides
        protocol.register_operation(self.opcode, self.defragment)               # register the operation in protocol

        # this dict holds the lists of received fragments:                      # actually it will reside in the "singleton"-class ReceivedFragments.lists dict
        # example:
        #  received_fragments = { <<message1_ID>>:
        #                           { "timestamp_last_append": <<datetime-object>>,
        #                             "total": <<total_fragments>>,
        #                             "fragment_list":
        #                               { <<fragment1ID>>: <<fragment1_data>>,
        #                                 <<fragment2ID>>: <<fragment2_data>>,
        #                                 ...
        #                               }
        #                           },
        #                         <<message2_ID>>:
        #                           { ...
        #                             ...
        #                           }
        #                       }
        self.received_fragments = ReceivedFragments().lists                     # link fragment_list from singleton into each clients Defragmentation instance
        threading.Thread.__init__(self)                                         # initialize Thread


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
## debug: show size of received_fragments
#        print "used memory of fragment list: ", sys.getsizeof(cPickle.dumps(self.received_fragments))
        
        now = datetime.now() # timestamp for 'now'

        if self.received_fragments != None:
            for id in self.received_fragments.keys() :                          # check for timed out lists (ALL fragment lists)
                time_diff = now - self.received_fragments[id]["timestamp_last_append"]
                if (time_diff.total_seconds() > self.fragment_timeout and       # use .total_seconds() instead of .seconds!
                    not self.received_fragments[id]["is_reconstructing"]):      # check if the list was completed and is currently being processed for reconstruction
                    log_msg = "fragment list " + str(id) + " timed out.."       # generate log message

                    if message["id"] != id:                                     # this check is just to make sure we don't remove a list
                        log_msg += " -> removing it.."                          #    that we would like to append to now..
                        del self.received_fragments[id]                         # remove the fragment list from the dict
                    else:
                        log_msg += " -> but we're just about to add fragment #"
                        log_msg += str(message.get("num")) + " of "
                        log_msg += str(self.received_fragments[message.get("id")]["total"])
                        log_msg += " ..keeping the list"
                    self.protocol.log("warning", log_msg)                       # log timeout to 'protocol' as 'warning'

        msg_opcode = message.get("op")                                          # check message fields: opcode, num, total, id, data
        msg_id = message.get("id")
        msg_num = message.get("num")
        msg_total = message.get("total")
        msg_data = message.get("data")

        if ((msg_opcode == None) or (msg_id == None) or                         # if any message field is missing
            (msg_num == None) or (msg_total == None) or                         #   -> abort
            (msg_data == None)):
            self.protocol.log("error", "received invalid fragment!")            # log invalid fragment as 'error'
            return                                                              #   ..this 'return' aborts processing of current fragment!

        log_msg = "fragment for messageID: " + str(msg_id) + " received."
        self.protocol.log("debug", log_msg)                                     # log reception of fragment as 'debug'

        if msg_id not in self.received_fragments.keys():                        # if not yet opened a fragment list for messageID,
            self.received_fragments[msg_id] = {}                                #    add entry for messageID to dict
            self.received_fragments[msg_id]["is_reconstructing"] = False        # this becomes true after all fragments for this message have been received, avoids removing of fragment lists that are currently being reconstructed
            self.received_fragments[msg_id]["total"] = message["total"]         # write announced total number of fragments to list
            self.received_fragments[msg_id]["timestamp_last_append"] = now      # add timestamp to list
            self.received_fragments[msg_id]["fragment_list"] = {}               # open a fragment list
                                                                                #   (this is actually a dict to allow easier access to fragments)
                                                                                #   (maybe use a simple list or chunks of fragments to get better
                                                                                #    performance with very large fragment lists) --> looks like performance is ok [2013.05.27 David Bertram]
                                                                                # !use String.join(..) instead of += !!
                                                                                #  -> create ordered list of fragments
                                                                                #  -> gather fragments that do not fit in a dict
                                                                                #     => always check if next fragment is in dict, if yes append to list
                                                                                #   --> finally use String.join() to reconstruct original message from ordered list of fragments..
            log_msg = "opened new fragment list for messageID " + str(msg_id)
            self.protocol.log("debug", log_msg)                                 # log new fragment list as 'debug'

        if (( msg_num not in self.received_fragments[msg_id]["fragment_list"].keys() ) and  # if current fragment not yet in list of fragments for current messageID
             msg_num <= self.received_fragments[msg_id]["total"] and                        #    and current num is ok with announced total number of fragments
             msg_total == self.received_fragments[msg_id]["total"]                          #    and announced total matches with 'message total'
            ):
            self.received_fragments[msg_id]["fragment_list"][msg_num] = msg_data   #   ..add it
            self.received_fragments[msg_id]["timestamp_last_append"] = now      # update timestamp for this list
            log_msg = "appended fragment #" + str(msg_num)
            log_msg += " (total: " + str(msg_total)+ ") to fragment list for messageID " + str(msg_id)
            self.protocol.log("debug", log_msg)                                 # log appended fragment as 'debug'
        else:
            log_msg = "error while trying to append fragment " + str(msg_num)
            self.protocol.log("error", log_msg)                                 # log problem while appending as 'error'
            return                                                              # return -> abort!
                                                                                # now check if fragment list is complete
        received_all_fragments = True                                           # watch out! ..preloading with True might be 'risky'
        existing_fragments = len(self.received_fragments[msg_id]["fragment_list"])
        announced_total = self.received_fragments[msg_id]["total"]
        if existing_fragments == announced_total:                               # if gathered all (only total count!) fragments
            log_msg = "enough/all fragments for messageID " + str(msg_id) + " received"
            log_msg += " [" + str(existing_fragments) + "]"
            self.protocol.log("debug", log_msg)                                 # log fragment list complete as 'debug'
            for i in range(0,announced_total):                                  # now double check, to be sure that every single fragment arrived
                if i not in self.received_fragments[msg_id]["fragment_list"]:   # if a fragment id is missing -> don't start reconstruction
                    received_all_fragments = False
                    log_msg = "fragment #" +str(i)
                    log_msg += " for messageID " + str(msg_id) + " is missing! "
                    self.protocol.log("error", log_msg)                         # log missing fragment as 'error'
        else:                                                                   # if not gathered all fragments -> don't start reconstruction
            received_all_fragments = False

        self.received_fragments[msg_id]["is_reconstructing"] = received_all_fragments

        if received_all_fragments:
            log_msg = "reconstructing original message " + str(msg_id)
            self.protocol.log("debug", log_msg)
            reconstructed_msg = ""                                              # now reconstruct original message
            for i in range(0,message["total"]):
                reconstructed_msg += self.received_fragments[msg_id]["fragment_list"][i]

            log_msg = "reconstructed original message:\n"
            log_msg += reconstructed_msg
            self.protocol.log("debug", log_msg)                                 # log reconstructed message as 'debug'
                                                                                # could check for valid JSON object before passing to protocol.incoming..
            duration = datetime.now() - now

            self.protocol.incoming(reconstructed_msg)                           # now try to pass reconstructed message to the 'protocol'
            log_msg = "reconstructed message (ID:" + str(msg_id) + ") from "
            log_msg += str(msg_total) + " fragments. "
            log_msg += "[message length: " + str(len(reconstructed_msg)) +"]"
            log_msg += "[duration: " + str(duration.total_seconds()) +  " s]"
            self.protocol.log("info", log_msg)                                  # log passed reconstructed message as 'info'

            del self.received_fragments[msg_id]                                 # remove fragment list after delivery to 'protocol'
            log_msg = "removed fragment list for messageID " + str(msg_id)
            self.protocol.log("debug", log_msg)


    # clean up
    def finish(self):
        self.received_fragments_for_msg_ID = None
        self.protocol.unregister_operation("fragment")