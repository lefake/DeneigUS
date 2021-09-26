import threading
import time
import rospy

from logging_utils import get_logger
from msg_utils import MsgFactory

class Topic:
    '''
    Use ros_obj when doing a One to One conversion (ex: Twist -> Twist)
    Use the obj_type only when changing the type (ex: Pose -> Float32MultiArray)
        The obj_type name should be the ROS name to work with the map
    '''
    def __init__(self, id, ros_obj, dst=None, obj_type=None):
        factory = MsgFactory()
        is_pub = True if isinstance(ros_obj, rospy.Publisher) else False
        class_name = ros_obj.data_class.__name__ if obj_type is None else obj_type
        converters = factory.getMsg(class_name)[1]

        self._id = id
        self._dst = dst
        self._name = ros_obj.name
        self._obj = factory.getMsg(class_name)[0]

        if is_pub:
            self._converter = converters[0]
        else:
            self._converter = converters[1]
        self._pub = ros_obj if is_pub else None

    @property
    def id(self):
        return self._id

    @property
    def dst(self):
        return self._dst

    @property
    def name(self):
        return self._name

    @property
    def obj(self):
        return self._obj

    @property
    def converter(self):
        return self._converter

    @property
    def pub(self):
        return self._pub

# Serialization utils
class PBSerializationHandler:
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

                    obj = current_topic.obj
                    obj.ParseFromString(bytearray.fromhex(raw_msg))
                    object_list.append([msg_id, obj])

        except Exception as e:
            self._logger.error("deserialize error " + str(e))

        return object_list




# Serial communication utils

class ArduinoReadHandler(threading.Thread):
    def __init__(self, sleeptime, readfunc):
        self._logger = get_logger("pb2ros.ArduinoReadHandler")
        self._logger.debug("ArduinoReadHandler started")
        self._sleeptime = sleeptime
        self._readfunc = readfunc
        threading.Thread.__init__(self)
        self._runflag = threading.Event()
        self._runflag.clear()
        self._run = True

    def run(self):
        self._runflag.set()
        self.worker()

    def worker(self):
        while self._run:
            if self._runflag.is_set():
                self._readfunc()
            time.sleep(self._sleeptime)

    def pause(self):
        self._runflag.clear()

    def resume(self):
        self._runflag.set()

    def running(self):
        return self._runflag.is_set()

    def kill(self):
        self._runflag.clear()
        self._run = False  


class PBSerialHandler:
    def __init__(self, serial, msg_callback, status_callback, serializer, sleeptime=0.01):
        self._logger = get_logger("pb2ros.PBSerialHandler")
        self._logger.debug("PBSerialHandler started")
        self._serial = serial
        self._sleeptime = float(sleeptime)
        self._msg_callback = msg_callback
        self._status_callback = status_callback

        self._interlock = False
        self._response = None

        self._serializer = serializer
        self._worker = ArduinoReadHandler(self._sleeptime, self.read_callback)
        self._worker.start()

    def kill(self):
        self._worker.kill()

    def read_callback(self):
        if not self._interlock:
            self._interlock = True
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
                    self._status_callback(self._response)

            except Exception as e:
                self._logger.error("Read call back error " + str(e))

            self._interlock = False

    def write_pb_msg(self, id, msg):
        self.write_pb_msgs([id], [msg])

    def write_pb_msgs(self, ids, msgs):
        encoded_msg = self._serializer.encode_msgs(ids, msgs)

        while self._interlock:
            time.sleep(self._sleeptime)

        self._interlock = True
        self._serial.write(encoded_msg.encode("ascii"))
        self._serial.flush()
        self._interlock = False

