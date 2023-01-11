import socket as skt
import time as tm

class Drone:
    def __init__(self, IP_ADDRESS, PORT, id):
        self.id = id
        self.sz = 1024
        self.com = skt.socket(skt.AF_INET, skt.SOCK_STREAM)
        self.com.connect((IP_ADDRESS, PORT))
        print("Pluto connected...")
        self.com.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 0)
        self.com.setsockopt(skt.IPPROTO_TCP, skt.TCP_KEEPCNT, 4)
        self.rc_raw_data = bytearray([36,77,60,16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234]) #header included
        self.set_cmd_data = bytearray([36,77,60,2,217,1,0,218]) #header included

    def update_checksum(self):
        checksum = 0
        for j in self.rc_raw_data[3:-1]:
            checksum ^= j
        self.rc_raw_data[21] = checksum
        checksum = 0
        for j in self.set_cmd_data[3:-1]:
            checksum ^= j
        self.set_cmd_data[7] = checksum

    def get_LSB_MSB(self, val):
        return bytearray([val % 256, val // 256])

    def arm(self):
        self.rc_raw_data[19] = 220
        self.rc_raw_data[20] = 5
        self.update_checksum()
        print("Pluto Armed...")
        self.com.send(self.rc_raw_data)
        tm.sleep(2)  # THIS WAS THE DAMN PROBLEM...absolutely need sleep after arm and disarm

    def disarm(self):
        self.rc_raw_data[19] = 176
        self.rc_raw_data[20] = 4
        self.update_checksum()
        print("Pluto Disarmed...")
        self.com.send(self.rc_raw_data)
        tm.sleep(2)   # THIS WAS THE DAMN PROBLEM...absolutely need sleep after arm and disarm

    def takeoff(self):
        self.set_cmd_data[5] = 1
        self.set_cmd_data[6] = 0
        self.update_checksum()
        self.com.send(self.set_cmd_data)

    def land(self):
        code_data = self.get_LSB_MSB(2)
        self.set_cmd_data[5] = code_data[0]
        self.set_cmd_data[6] = code_data[1]
        self.update_checksum()
        self.com.send(self.set_cmd_data)

    def throttle(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[9] = data[0]
        self.rc_raw_data[10] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def pitch(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[7] = data[0]
        self.rc_raw_data[8] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def roll(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[5] = data[0]
        self.rc_raw_data[6] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def yaw(self, val):
        data = bytearray(self.get_LSB_MSB(val))
        self.rc_raw_data[11] = data[0]
        self.rc_raw_data[12] = data[1]
        self.update_checksum()
        self.com.sendall(self.rc_raw_data)

    def disconnect(self):
        self.com.close()
        tm.sleep(1)
        print('Pluto disconnected')

def throttle_test(time):
    drone = Drone("192.168.4.1", 23, 1)
    drone.arm()
    drone.takeoff()
    clock_start = tm.time()
    while(tm.time()-clock_start < time):
        drone.throttle((1100))
        tm.sleep(0.022)
    drone.land()
    drone.disarm()
    drone.disconnect()

try:
  throttle_test(5)
except ConnectionAbortedError:
  throttle_test(5)
