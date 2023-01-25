from threading import Thread
import socket as skt
import time


class Drone:
   
    def __init__(self, IP_ADDRESS, PORT, id, debug=False):
        self.id = id
        self.com = skt.socket(skt.AF_INET, skt.SOCK_STREAM)
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = PORT
        self.rc_raw_data = bytearray([36,77,60,16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234]) #header included
        self.set_cmd_data = bytearray([36,77,60,2,217,1,0,218]) #header included
        self.cmd_set = False
        self.is_armed = False
        self.is_stopped = False
        self.is_skt_connected = False
        self.debug = debug

    def connect(self):
        Thread(target=self.__com_thread, args=()).start()
        return self

    def disconnect(self):
        self.is_stopped = True
        if self.is_skt_connected:
            self.com.close()
        print('Disconnected...')

    def __com_thread(self):
        if self.debug: print('Thread Started')
        try:
            self.com.connect((self.IP_ADDRESS, self.PORT))
            self.com.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 0)
            self.com.setsockopt(skt.IPPROTO_TCP, skt.TCP_KEEPCNT, 4)
            self.is_skt_connected = True
        except:
            print("Couldn't connect...")
            return
        print("Connected...")
        try:
            self.__update()
        except:
            # update error message
            print('Connection lost...retrying...')
            self.com_thread()

    def __update(self):
        while True:
            if self.is_stopped:
                if self.debug: print("Thread Stopped")
                return
            if self.cmd_set:
                self.set_cmd()
                self.cmd_set = False
            else:
                self.__set_rc_raw()
            time.sleep(0.022)
            

    def __update_checksum(self):
        checksum = 0
        for j in self.rc_raw_data[3:-1]:
            checksum ^= j
        self.rc_raw_data[21] = checksum
        checksum = 0
        for j in self.set_cmd_data[3:-1]:
            checksum ^= j
        self.set_cmd_data[7] = checksum

    def __get_LSB_MSB(self, val):
        return bytearray([val % 256, val // 256])

    def __set_rc_raw(self):
        self.__update_checksum()
        self.com.sendall(self.rc_raw_data)

    def set_cmd(self):
        self.__update_checksum()
        self.com.sendall(self.set_cmd_data)

    def arm(self):
        self.rc_raw_data[19] = 220
        self.rc_raw_data[20] = 5
        if self.debug: print("Armed...")

    def disarm(self):
        self.rc_raw_data[19] = 176
        self.rc_raw_data[20] = 4
        if self.debug: print("Disarmed...")

    def takeoff(self):
        if self.debug: print("Takeoff...")
        self.arm()
        self.set_cmd_data[5] = 1
        self.set_cmd_data[6] = 0
        self.cmd_set = True

    def land(self):
        if self.debug: print("Landing...")
        self.set_cmd_data[5] = 2
        self.set_cmd_data[6] = 0
        self.cmd_set = True
        time.sleep(3)
        self.disarm()

    def set_throttle(self, val):
        if self.debug: print(f"Set Throttle to {val}")
        data = bytearray(self.__get_LSB_MSB(val))
        self.rc_raw_data[9] = data[0]
        self.rc_raw_data[10] = data[1]

    def set_pitch(self, val):
        if self.debug: print(f"Set Pitch to {val}")
        data = bytearray(self.__get_LSB_MSB(val))
        self.rc_raw_data[7] = data[0]
        self.rc_raw_data[8] = data[1]

    def set_roll(self, val):
        if self.debug: print(f"Set Roll to {val}")
        data = bytearray(self.__get_LSB_MSB(val))
        self.rc_raw_data[5] = data[0]
        self.rc_raw_data[6] = data[1]

    def set_yaw(self, val):
        if self.debug: print(f"Set Yaw to {val}")
        data = bytearray(self.__get_LSB_MSB(val))
        self.rc_raw_data[11] = data[0]
        self.rc_raw_data[12] = data[1]

    def set_state(self, throttle, pitch, roll, yaw=1500):
        # if self.debug: print(f"Set States to T: {throttle}, P: {pitch}, R: {roll}, Y: {yaw}")
        throttle_data = bytearray(self.__get_LSB_MSB(throttle))
        pitch_data = bytearray(self.__get_LSB_MSB(pitch))
        roll_data = bytearray(self.__get_LSB_MSB(roll))
        yaw_data = bytearray(self.__get_LSB_MSB(yaw))
        self.rc_raw_data[9] = throttle_data[0]
        self.rc_raw_data[10] = throttle_data[1]
        self.rc_raw_data[7] = pitch_data[0]
        self.rc_raw_data[8] = pitch_data[1]
        self.rc_raw_data[5] = roll_data[0]
        self.rc_raw_data[6] = roll_data[1]
        self.rc_raw_data[11] = yaw_data[0]
        self.rc_raw_data[12] = yaw_data[1]
    