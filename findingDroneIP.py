# # A basic script to scan a local network for IP addresses to indentify Tello EDU drones

# # Import modules
# import subprocess
# import ipaddress
# from subprocess import Popen, PIPE

# # Create the network
# # The IP below is associated with the TP-Link wireless router
# # https://amzn.to/2TR1r56
# ip_net = ipaddress.ip_network(u'192.168.137.1/24', strict=False)

# # Loop through the connected hosts
# for ip in ip_net.hosts():

#     # Convert the ip to a string so it can be used in the ping method
#     ip = str(ip)
    
#     # Let's ping the IP to see if it's online
#     toping = Popen(['ping', ip], stdout=PIPE)
#     output = toping.communicate()[0]
#     hostalive = toping.returncode
    
#     # Print whether or not device is online
#     if hostalive ==0:
#         print(ip, "is online")
#         print("--------------------------------------------------------------------------------")
#     else:
#         print(ip, "is offline")

# # '-c', '1', '-W', '50',


import socket

def port_scanner(host, port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex((host, port))
        if result == 0:
            print("Port {} is open".format(port))
        sock.close()
    except:
        print("Unable to scan the port")

host = "192.168.137.139"

for port in range(1, 65535):
    port_scanner(host, port)
