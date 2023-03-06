from Communication import Drone
import time

def com_test():
    drone = Drone("192.168.4.1", 23, 5, debug=True) #creates drone object, set debug to true to get console output on every action.
    drone.connect() #starts looping in a separate thread
    time.sleep(1)
    drone.takeoff()
    drone.set_state(1500, 1500, 1500) #can simultaneously set all state values (TPRY) or (TPR)
    time.sleep(4)
    drone.land()
    time.sleep(4)

    #Note : disarm after land and rearm before takeoff is mandatory.
    # so..integrated arm and disarm inside takeoff and land.
    drone.takeoff()
    drone.set_state(1500, 1500, 1500)
    time.sleep(4)
    drone.land()
    time.sleep(1)
    drone.disconnect()

com_test()
