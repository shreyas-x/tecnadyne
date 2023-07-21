#!/usr/bin/env python3
import copy
import crcmod

START_BYTE = 1
END_BYTE = 2
START_OF_PACKET = 119
END_OF_PACKET = 10
class Decoder():

    def __init__(self):
        self._state = START_BYTE
        self._arr = bytearray()
        
        self.crc = crcmod.predefined.mkPredefinedCrcFun("crc-8")
        self._func_dic = { START_BYTE: self.on_start,
                           END_BYTE : self.on_end}

    def parse_bytes(self, arr):
        packets = []
        for i in range(len(arr)):
             pkts = self.parse_byte(arr[i:i+1])
             if pkts is not None:
                 packets.append(pkts)
        return packets

    def decode(self, packet):

        if (not self.checksum(packet)):
            return None, None
        #Convert to string
        pkt = packet.decode("utf-8") 
        data = pkt.split(",")

        if data[0] == "wrz":
            return 1, self.parse_wrz(data)
        if data[0] == "wru":
            return 2, self.parse_wru(data)
        if data[0] == "wrp":
            return 3, self.parse_wrp(data)
        if data[0] == "wrc":
            return 4, self.parse_config(data)

        return None, None

    def parse_config (self, data):
        fin_data = data[4].split("*")
        return {
            "speed_of_sound" : float(data[1]),
            "mounting_rot_offset" : float(data[2]),
            "acoustic_enable" : True if data[3] == "y" else False,
            "dark_mode_enable" : True if fin_data[0] == "y" else False,
        }

    def parse_wrz(self, data):
        fin_data = data[11].split("*")
        cov_data = data[7].split(";")
        cov = [float(x) for x in cov_data]
        return {
            "vel_x" : float(data[1]),
            "vel_y" : float(data[2]),
            "vel_z" : float(data[3]),
            "valid" : True if data[4] == "y" else False,
            "altitude" : float(data[5]),
            "fom": float(data[6]),
            "covariance": cov,
            "time_of_validity": float(data[8]),
            "time_of_transmission" : float(data[9]),
            "time" : float(data[10]),
            "status" : True if fin_data[0] == "1" else False
        }

    def parse_wru(self, data):
        fin_data = data[5].split("*")
        return {
           "beam_id" : int(data[1]),
           "velo" : float(data[2]),
           "distance" : float(data[3]),
           "rssi" : float(data[4]),
           "nsd" : float(fin_data[0])
        }

    def parse_wrp(self, data):
        fin_data = data[9].split("*")
        return {
           "time_stamp" : float(data[1]),
           "x_dist" : float(data[2]),
           "y_dist" : float(data[3]),
           "z_dist" : float(data[4]),
           "pos_std" : float(data[5]),
           "roll" : float(data[6]),
           "pitch" : float(data[7]),
           "yaw" : float(data[8]),
           "status" : float(fin_data[0])
        }

    def on_start(self, byte):
        if byte[0] == START_OF_PACKET:
            self.clear()
            self._arr += byte
            self._state = END_BYTE

    def on_end(self,byte):
        if byte[0] == END_OF_PACKET:
            pkt = copy.deepcopy(self._arr)
            self.clear()
            self._state = START_BYTE
            return pkt

        return None

    def clear(self):
        self._state = START_BYTE
        self._arr = bytearray()

    def parse_byte(self, byte):
        assert len(byte) == 1
        self._arr += byte
        return self._func_dic[self._state](byte)

    def checksum(self, sentence):
        data, checksum = sentence[:-2].split(b"*")
        if self.crc(data) == int(checksum, 16):
            return True
        return False
