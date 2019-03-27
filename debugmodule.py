"""
This module is for the debug info in athena
"""
import socket
import proto.zss_debug_pb2 as athena_dbg_pb


class DebugModule:
    """
    At start there will be a rectangle wrapping the area for debug info.
    The left top corner: (-600, -450)
    """
    def __init__(self, ACTION_IP = '127.0.0.1', DEBUG_PORT = 20001):
        """
        The debug port for Athena is 20001 and IP address is '127.0.0.1'
        :param ACTION_IP: destination IP
        :param ACTION_PORT: destination port
        """
        self.address = (ACTION_IP, DEBUG_PORT)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def msg_init(self):
        self.messages = athena_dbg_pb.Debug_Msgs()

    def dwa_msg_init(self):
        """
        For now, we need 3 types of messages:
        pos.
        control signal.
        final cost.
        :return: None
        """
        self.messages = athena_dbg_pb.Debug_Msgs()
        self.dwa_msgs = [self.messages.msgs.add() for i in range(5)]
        for i in range(4):
            self.dwa_msgs[i].type = 2
            self.dwa_msgs[i].color = 0
            text_msg = self.dwa_msgs[i].text
            text_msg.pos.x = -600
            text_msg.pos.y = -450 + 20*(i+1)
        self.dwa_msgs[0].text.text = "POS:"
        self.dwa_msgs[1].text.text = "U:"
        self.dwa_msgs[2].text.text = "FINAL COST:"
        self.dwa_msgs[3].text.text = "ROUND:"

        # The 5th is a line
        self.dwa_msgs[4].type = 1
        self.dwa_msgs[4].color = 0
        self.dwa_msgs[4].line.FORWARD = True
        self.dwa_msgs[4].line.BACK = False

    def send_msg(self, messages):
        self.socket.sendto(messages.SerializeToString(), self.address)

    def add_text(self, x, y, text):
        message = self.messages.msgs.add()
        message.type = 2
        message.color = 0

        text_msg = message.text
        pos = text_msg.pos
        pos.x = x
        pos.y = y

        text_msg.text = text

    def add_line(self, start_x, start_y, end_x, end_y, forward):
        point1 = athena_dbg_pb.Point()
        point2 = athena_dbg_pb.Point()
        point1.x = start_x
        point1.y = start_y
        point2.x = end_x
        point2.y = end_y

        message = self.messages.msgs.add()
        message.type = athena_dbg_pb.Debug_Msg.LINE
        message.color = athena_dbg_pb.Debug_Msg.WHITE

        line = athena_dbg_pb.Debug_Line()
        line.start.CopyFrom(point1)
        line.end.CopyFrom(point2)
        line.FORWARD = True
        line.BACK = False
        message.line.CopyFrom(line)


if __name__ == "__main__":
    debug = DebugModule('127.0.0.1', 20001)
    debug.dwa_msg_init()
    # debug.add_line(-460, -352, 0, 0, True)
    # debug.add_text(-500, -400, "hello world")

    debug.send_msg(debug.messages)
