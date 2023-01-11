import socket as skt,time as tm

sz=1024
myDrone=skt.socket(skt.AF_INET, skt.SOCK_STREAM)
myDrone.connect(("192.168.4.1" , 23))
myDrone.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 1)
print('Connected')
tm.sleep(5)
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
    myDrone.send(pack_dat)

def throttle(val):
    dat=bytearray(get_LSB_MSB(val))
    pack_dat[9]=dat[0]
    pack_dat[10]=dat[1]
    pack_dat[21]=update_checksum(pack_dat)
    myDrone.send(pack_dat)

def takeoff():
    dat = bytearray( [ 2,217,1,0,1 ] )
    packet = header[:]
    packet.extend(dat)
    packet[7]=update_checksum(packet)
    myDrone.send(packet)

def land():
    dat = bytearray( [ 2,217,2,0,1 ] )
    packet = header[:]
    packet.extend(dat)
    packet[7]=update_checksum(packet)
    myDrone.send(packet)

def disarm():
    dat = pack_dat[:]
    dat[19]=176
    dat[20]=4
    dat[21] = update_checksum(dat)
    myDrone.send(dat)

    

arm()
tm.sleep(2)
land()
print("Landed")
tm.sleep(2)
disarm()
print("Disarmed")
tm.sleep(3)
myDrone.close()
print("Disconnected")