import socket as skt,time as tm

# server = skt.socket(skt.AF_INET, skt.SOCK_STREAM , skt.IPPROTO_TCP)
# server.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 0)
# server.setsockopt(skt.IPPROTO_TCP, skt.TCP_KEEPCNT, 4)
# server.connect(("192.168.137.125" , 23))


# takeoff_arr = [36,77,60,2,217,1,0,218]
# state_arr = [36,77,60,16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234]

# server.sendall(bytearray(takeoff_arr))
# response = server.recv(1024)
# print(str(response))

sz=1024
myDrone=skt.socket(skt.AF_INET, skt.SOCK_STREAM , skt.IPPROTO_TCP)
myDrone.connect(("192.168.137.125" , 23))

print('Connected')
tm.sleep(3)
header=bytearray([ord('$') , ord('M') , ord('<')])

#MSP_SET_RAW_RC
init_val = bytearray([16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234])
pack_dat = header[:]
pack_dat.extend(init_val[:])

# print(pack_dat)

def update_checksum(packet):
    xor_arr=packet[3:-1]
    i=0
    for j in xor_arr:
        i^=j
    return i

def get_LSB_MSB(val):
    return bytearray( [val%256 , val//256] )

def arm():           
    pack_dat[19]=220
    pack_dat[20]=5
    pack_dat[21]=update_checksum(pack_dat)
    print("Armed")
    myDrone.sendall(pack_dat)

def throttle(val):
    dat=bytearray(get_LSB_MSB(val))
    pack_dat[9]=dat[0]
    pack_dat[10]=dat[1]
    pack_dat[21]=update_checksum(pack_dat)
    myDrone.sendall(pack_dat)

def takeoff():
    dat = bytearray( [ 2,217,1,0,1 ] )
    packet = header[:]
    packet.extend(dat)
    packet[7]=update_checksum(packet)
    myDrone.sendall(packet)

def land():
    dat = bytearray( [ 2,217,2,0,1 ] )
    packet = header[:]
    packet.extend(dat)
    packet[7]=update_checksum(packet)
    myDrone.sendall(packet)

def disarm():
    dat = pack_dat[:]
    dat[19]=176
    dat[20]=4
    dat[21] = update_checksum(dat)
    myDrone.sendall(dat)

disarm()
tm.sleep(2)
arm()
tm.sleep(2)
takeoff()
tm.sleep(1)
disarm()
tm.sleep(2)
myDrone.close()

# server = skt.socket(skt.AF_INET, skt.SOCK_STREAM , skt.IPPROTO_TCP)
# server.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 0)
# server.setsockopt(skt.IPPROTO_TCP, skt.TCP_KEEPCNT, 4)
# server.connect(("192.168.137.125" , 23))


# takeoff_arr = [36,77,60,2,217,1,0,218]
# state_arr = [36,77,60,16,200,220,5,220,5,220,5,220,5,176,4,232,3,220,5,176,4,234]

# server.sendall(bytearray(takeoff_arr))
# response = server.recv(1024)
# print(str(response))