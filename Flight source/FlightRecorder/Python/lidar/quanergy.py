import numpy as np
import socket
import threading
import time

class QuanergyPacketReader(object):
    pts_per_packet = 50
    expected_packet_size = 6632  #bytes

    header = np.dtype([
                       ('signature', '>u4'),
                       ('message_size', '>u4'),  # 6632
                       ('timestamp_s', '>u4'),
                       ('timestamp_ns', '>u4'),
                       ('api_v0', '>u1'),
                       ('api_v1', '>u1'),
                       ('api_v2', '>u1'),
                       ('packet_type', '>u1'),
                      ])

    firing_data = np.dtype([
                            ('rotation', '>u2'),
                            ('reserved', '>u2'),
                            ('distance', '>u4', (3, 8)),
                            ('intensity', '>u1', (3, 8)),
                            ('status', '>u1', 8)
                           ])

    packet_data = np.dtype([
                            ('firing_data', firing_data, (pts_per_packet, )),
                            ('timestamp_s', '>u4'),
                            ('timestamp_ns', '>u4'),
                            ('api_v', '>u2'),
                            ('status', '>u2')
                           ])

    packet = np.dtype([
                      ('header', header), 
                      ('data', packet_data)
                      ])

    firing_angles = np.array([
                            -0.318505,
                            -0.2692,
                            -0.218009,
                            -0.165195,
                            -0.111003,
                            -0.0557982,
                             0.0,
                             0.0557982
                             ])
    firing_angles_sin = np.sin(firing_angles)
    firing_angles_cos = np.cos(firing_angles)

    def read_packet(self, packet):
        packet = np.frombuffer(packet, self.packet, 1)
        return packet

    def read_packet_as_xyzdi(self, packet, xyzdi=None):
        packet = self.read_packet(packet)
        return self.packet2xyzdi(packet[0]['data']['firing_data'], xyzdi)

    def packet2xyzdi(self, data, xyzdi=None):
        ppp = self.pts_per_packet
        if xyzdi is None:
            xyzdi = np.zeros([ppp, 8, 3], float)
        rotation_radians = 2 * np.pi * data['rotation'] / 10400.
        distances = data['distance'][:, 0, :]
        intensity = data['intensity'][:, 0, :]
        if (distances.shape[0] != ppp):
            raise Exception("Packet size %d doesn't match expected number %d"\
                % (distances.shape[0], ppp))
        xydistances = distances * self.firing_angles_cos
        xyzdi[:ppp, :, 0] = np.cos(rotation_radians)[:, None] * xydistances
        xyzdi[:ppp, :, 1] = np.sin(rotation_radians)[:, None] * xydistances
        xyzdi[:ppp, :, 2] = distances * self.firing_angles_sin
        xyzdi[:ppp, :, 3] = distances
        xyzdi[:ppp, :, 4] = intensity

        return xyzdi

class Quanergy(object):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    polling = True
    qpr = QuanergyPacketReader()
    laser_fire_rate = 53828
    rpm = 10  # Hz
    xyzdi = np.zeros([(laser_fire_rate / rpm * 4 // 50) * 50, 8, 5])
    packet = None
    buf = None

    def __init__(self, tcp_ip, tcp_port):
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port

    def start_polling(self):
        qpr = self.qpr
        s = self.s
        s.connect((self.tcp_ip, self.tcp_port))

        self.polling = True
        def poll_buffer():
            self.buf = bytearray(QuanergyPacketReader.expected_packet_size)
            i = 0
            ppp = qpr.pts_per_packet
            while self.polling:
                view = memoryview(self.buf)
                bytestoread = QuanergyPacketReader.expected_packet_size
                while bytestoread:
                    nbytes = s.recv_into(view, bytestoread)
                    view = view[nbytes:]
                    bytestoread -= nbytes

                self.packet = qpr.read_packet(self.buf)
                if self.packet[0]['data']['status'] != 0:
                    continue
                self.xyzdi[i:i + ppp, :, :] = \
                      qpr.packet2xyzdi(self.packet[0]['data']['firing_data'],
                                       self.xyzdi[i:i + ppp, :, :])
                i = (i + ppp) * ((i + ppp) < self.xyzdi.shape[0])
                time.sleep(1 / self.laser_fire_rate / 2.)

        t = threading.Thread(target=poll_buffer)
        t.start()

    def stop_polling(self):
        self.polling = False
        self.s.close()

    def __del__(self):
        self.stop_polling()

if __name__ == '__main__':
    TCP_IP = '10.11.34.2'
    TCP_PORT = 4141

    q8 = Quanergy(TCP_IP, TCP_PORT)
    q8.start_polling()
    time.sleep(50)
    q8.stop_polling()
    print(repr(q8.xyzdi))