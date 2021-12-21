import threading
import time
import copy

from logging_utils import get_logger
from msg_utils import default_converters

millis = lambda: int(time.time() * 1000)

class Topic:
    def __init__(self, id, name, converter):
        self._id = id
        self._name = name
        self._converter = converter

    @property
    def id(self):
        return self._id

    @property
    def name(self):
        return self._name

    @property
    def converter(self):
        return self._converter

class PubTopic(Topic):
    def __init__(self, id, ros_pub, input_type=None, converter=None):
        if input_type is None:
            input_type = default_converters[ros_pub.data_class.__name__][0]

        if converter is None:
            converter = default_converters[ros_pub.data_class.__name__][1][0]

        super().__init__(id, ros_pub.name, converter)
        self._input_type = input_type
        self._pub = ros_pub

    @property
    def input_type(self):
        return self._input_type

    @property
    def pub(self):
        return self._pub

class SubTopic(Topic):
    def __init__(self, id, ros_sub, arduino_dst, converter=None):
        if converter is None:
            converter = default_converters[ros_sub.data_class.__name__][1][1]

        super().__init__(id, ros_sub.name, converter)
        self._dst = arduino_dst

    @property
    def dst(self):
        return self._dst

# Serialization utils
class PBSerializationHelper:
    def __init__(self, topics):
        self._logger = get_logger("pb2ros.PBSerializationHandler")
        self._logger.debug("PBSerializationHandler started")
        self._topics = topics

    def encode_msgs(self, ids, msgs):
        msg = "<"

        for id_msg, pb_msg in zip(ids, msgs):
            msg += str(id_msg) + "|"
            for byte in bytearray(pb_msg.SerializeToString()):
                msg += str(hex(byte))[2:].zfill(2)  # Remove \x and fill with 0 in front to always takes 2 digits
            msg += ";"

        msg += ">"
        return msg

    def encode_msg(self, id, msg):
        return self.encode_msgs([id], [msg])

    def deserialize(self, messages):
        object_list = []
        try:
            messages = messages.decode("ascii")
            msg_array = messages[1:-1].split(';')      # Remove < > characters and split sub-msgs

            for msg in msg_array:
                if len(msg) > 0:
                    msg_id, raw_msg = msg.split("|")    # Find the id of the message
                    msg_id = int(msg_id)
                    current_topic = next((topic for topic in self._topics if topic.id == msg_id), None)

                    if current_topic is None:
                        self._logger.error("Unknown topic id: " + str(msg_id))
                        continue

                    input_type = current_topic.input_type
                    input_type.ParseFromString(bytearray.fromhex(raw_msg))
                    object_list.append([msg_id, input_type])

        except Exception as e:
            self._logger.error("deserialize error " + str(e))

        return object_list

class TimedBuffer(threading.Thread):
    def __init__(self, nbs_msg_max, send_callback, d=100):
        self._logger = get_logger("TimedBuffer.main")
        self._logger.debug("Started TimedBuffer init")
        threading.Thread.__init__(self)

        self.msgs = {}
        self.nbs_msg_max = nbs_msg_max
        self.is_ready_to_send = False

        self.last_time = millis()
        self.delay = d
        self.send_callback = send_callback
        self.lock = False
        self.running = False

    def add(self, name, val):
        while self.running and self.lock:
            pass

        self.lock = True
        self.is_ready_to_send = self.msgs.__contains__(name) # Could be dependant on topic name
        self.msgs[name] = val  # Will override but trigger a send
        self.is_ready_to_send = self.is_ready_to_send or len(self.msgs) == self.nbs_msg_max
        self.lock = False

    def watcher(self):
        while self.running:
            if len(self.msgs) != 0 and (self.is_ready_to_send or (millis() - self.last_time) > self.delay):
                while self.running and self.lock:
                    pass

                self.lock = True
                msgs_copy = copy.copy(self.msgs)
                self.msgs.clear()
                self.is_ready_to_send = False
                self.lock = False
                
                self.send_callback(list(msgs_copy.keys()), list(msgs_copy.values()))
                self.last_time = millis()
            else:
                time.sleep(0.001)

    def run(self):
        self.running = True
        self.watcher()

    def kill(self):
        self.running = False

class PBSerialHandler(threading.Thread):
    id_list = []
    def __init__(self, serial, msg_callback, status_callback, id_error_callback, serializer, ack_query, sleeptime=0.01):
        threading.Thread.__init__(self)
        self._logger = get_logger("pb2ros.PBSerialHandler")
        self._logger.debug("PBSerialHandler started")

        self._serial = serial
        self._msg_callback = msg_callback
        self._status_callback = status_callback
        self._id_error_callback = id_error_callback
        self._serializer = serializer
        self._sleeptime = sleeptime
        self._ack_query = ack_query

        self.timedBuffer = TimedBuffer(6, self.__write_pb_msgs) 
        self.timedBuffer.start()

        self._id = ""
        self._interlock = False
        self._response = None
        self._run = False

        # Need to make sure the thread is reading 
        # the inputs before sending anything
        # otherwise the communication is bricked
        time.sleep(1)
        self.acknowledge_arduino()

    @property
    def id(self):
        return self._id

    def acknowledge_arduino(self):
        while self._interlock:
            pass

        self._interlock = True
        self._to_send = self._ack_query
        self._interlock = False

    def write_msg(self, id, msg):
        self.timedBuffer.add(id, msg)

    def __write_pb_msgs(self, ids, msgs):
        encoded_msg = self._serializer.encode_msgs(ids, msgs)

        while self._interlock:
            pass

        self._interlock = True
        self._to_send = encoded_msg
        self._interlock = False

    def run(self):
        self._run = True
        self.worker()

    def kill(self):
        self._run = False  
        self.timedBuffer.kill()

    def worker(self):
        while self._run:
            # There's something to read
            try:
                if self._serial.in_waiting > 0:     
                    try:
                        input = self._serial.read()

                        if input == b'<':
                            buffer = self._serial.read_until(b'>')
                            self._serial.flush()
                            self._response = b'<' + buffer
                            self._msg_callback(self._response)

                        elif input == b'{':
                            buffer = self._serial.read_until(b'}')
                            self._serial.flush()
                            self._response = b'{' + buffer

                            if len(self._response.decode()[1:-1].split(';')) == 1:
                                self._id = self._response.decode()[1:-1]
                                self.id_list.append(self._id)
                                self._logger.info(f"Ack to {self._id}")

                                # Test if duplicate in list
                                if len(self.id_list) != len(set(self.id_list)):
                                    self._id_error_callback()
                            else:
                                self._status_callback(self._id, self._response)

                    except Exception as e:
                        self._logger.error("Read call back error " + str(e))

                # There's something to write
                elif self._to_send != "" and not self._interlock:
                    self._serial.write(self._to_send.encode("ascii"))
                    self._serial.flush()
                    self._interlock = True
                    self._to_send = ""
                    self._interlock = False

                time.sleep(self._sleeptime)
            except Exception as e:
                pass
