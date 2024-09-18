import logging
import time
import sys
import socket
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Define the UDP IP address and port to listen on
# Use IP address of host running Python
UDP_IP = "192.168.1.199"
UDP_PORT = 8889

# Set Uniform Resource Identifier and default height constant
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

battery_level = 0
pm_state = 0
batt_state_ticks = 0
batt_green_time = 150
batt_yellow_time = 150
batt_green_exp = 0
batt_yellow_exp = 0

def batt_led():
    global batt_state_ticks, batt_green_time, batt_yellow_time, batt_green_exp, batt_yellow_exp
    if (pm_state == 3) or (battery_level == 0): return 3
    if (battery_level >= 20):
        if batt_yellow_exp: return 2
        elif batt_green_exp: return 1
        elif batt_state_ticks < batt_green_time: return 0
        else:
            batt_green_exp = 1
            batt_state_ticks = 0
            return 1
    if (battery_level == 10):
        if batt_yellow_exp: return 2
        elif (batt_state_ticks < batt_green_time) and (batt_green_exp == 0):
            batt_green_exp = 1
            batt_state_ticks = 0
            return 1
        elif batt_state_ticks < batt_yellow_time: return 1
        else:
            batt_yellow_exp = 1
            batt_green_exp = 1
            batt_state_ticks = 0
            return 2

def log_batt_callback(timestamp, data, logconf):
    # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global battery_level, pm_state, batt_state_ticks
    battery_level = data['pm.batteryLevel']
    pm_state = data['pm.state']
    batt_state_ticks += 1
    # print(battery_level)

def move_with_EEK(scf):
    """
    Execute commands received via UDP from EEK.

    """  
    commander = scf.cf.high_level_commander      
    mc = MotionCommander(scf, default_height=DEFAULT_HEIGHT)    
    while True:
    # Listen for commands
        data, addr = sock.recvfrom(1024)
        decodedData = data.decode('utf-8').rstrip('\x00')
        if '_v' not in decodedData:
            print(f"Received packet from {addr}: {decodedData}")        
        packetPieces = decodedData.split(' ')
        try:
            if 'battery' in decodedData:
                battLED = batt_led()
                sock.sendto(("battery: " + str(battLED)).encode('utf-8'), addr)
                if (battLED == 3):
                    print("shutting down: low battery")
                    break
                print(batt_state_ticks)
                print(pm_state)
                print(battery_level)
                time.sleep(0.1)
            elif 'stop_v' in decodedData:
                mc.stop()
            elif 'takeoff' in decodedData:
                beforeTOBatt = batt_led()
                if (beforeTOBatt == 3):
                    print("shutting down: low battery")
                    break
                else:
                    mc.take_off(0.5)
                    time.sleep(0.5)
                    # sock.sendto("takeoff ok".encode('utf-8'), addr)
                    sock.sendto(("battery: " + str(beforeTOBatt)).encode('utf-8'), addr)
            elif 'land' in decodedData:
                mc.land()
                time.sleep(2.5)
                afterLandBatt = batt_led()
                # sock.sendto("land ok".encode('utf-8'), addr)
                sock.sendto(("battery: " + str(afterLandBatt)).encode('utf-8'), addr)
                print(batt_state_ticks)
                print(battery_level)
            elif 'up_v' in decodedData:
                mc.start_up(int(packetPieces[1])/100)
            elif 'up' in decodedData:
                mc.up(0.5)
                time.sleep(0.5)
                sock.sendto("up ok".encode('utf-8'), addr)
            elif 'down_v' in decodedData:
                mc.start_down(int(packetPieces[1])/100)
            elif 'down' in decodedData:
                mc.down(0.5)
                time.sleep(0.5)
                sock.sendto("down ok".encode('utf-8'), addr)
            elif 'forward' in decodedData:
                mc.forward(0.8)
                time.sleep(0.5)
                sock.sendto("forward ok".encode('utf-8'), addr)
            elif 'back' in decodedData:
                mc.back(0.8)
                time.sleep(0.5)
                sock.sendto("back ok".encode('utf-8'), addr)
            elif 'right' in decodedData:
                mc.right(0.8)
                time.sleep(0.5)
                sock.sendto("right ok".encode('utf-8'), addr)
            elif 'left' in decodedData:
                mc.left(0.8)
                time.sleep(0.5)
                sock.sendto("left ok".encode('utf-8'), addr)
            elif 'CCW_circ' in decodedData:
                mc.circle_left(int(packetPieces[1])/100,
                                int(packetPieces[2])/100,
                                int(packetPieces[3]))
                time.sleep(0.5)
                sock.sendto("left circle ok".encode('utf-8'), addr)
            elif 'CW_circ' in decodedData:
                mc.circle_right(int(packetPieces[1])/100,
                                int(packetPieces[2])/100,
                                int(packetPieces[3]))
                time.sleep(0.5)
                sock.sendto("right circle ok".encode('utf-8'), addr)
            elif 'go_fo_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,0,0)
            elif 'go_ba_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,0,0)
            elif 'go_le_v' in decodedData:
                mc.start_linear_motion(0,int(packetPieces[2])/100,0)
            elif 'go_ri_v' in decodedData:
                mc.start_linear_motion(0,int(packetPieces[2])/100,0)
            elif 'go_fr_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,int(packetPieces[2])/100,0)
            elif 'go_fl_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,int(packetPieces[2])/100,0)
            elif 'go_br_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,int(packetPieces[2])/100,0)
            elif 'go_bl_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,int(packetPieces[2])/100,0)
            elif 'go_v' in decodedData:
                mc.start_linear_motion(int(packetPieces[1])/100,
                                    int(packetPieces[2])/100,
                                    int(packetPieces[3])/100,
                                    int(packetPieces[4]))
            elif 'ccw_v' in decodedData:
                mc.start_turn_left(int(packetPieces[1]))
                # mc.start_turn_left()
            elif 'ccw' in decodedData:
                mc.turn_left(90)
                time.sleep(0.5)
                sock.sendto("ccw ok".encode('utf-8'), addr)
            elif 'cw_v' in decodedData:
                mc.start_turn_right(int(packetPieces[1]))
                # mc.start_turn_right()
            elif 'cw' in decodedData:
                mc.turn_right(90)
                time.sleep(0.5)
                sock.sendto("cw ok".encode('utf-8'), addr)
            elif 'emergency' in decodedData:
                sock.sendto("Kill!".encode('utf-8'), addr)
                commander.stop() # should stop motors
                break
            else:
                time.sleep(0.1)
        except Exception as error:
            print("Exception: ", error)
            break

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

if __name__ == '__main__':
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}")
        
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Run flight function
        # A crazyflie must be connected to radio
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconfBatt = LogConfig(name='Battery', period_in_ms=1000)
        logconfBatt.add_variable('pm.batteryLevel')
        logconfBatt.add_variable('pm.state')
        scf.cf.log.add_config(logconfBatt)
        logconfBatt.data_received_cb.add_callback(log_batt_callback)

        logconfBatt.start()
        time.sleep(2)
        print(battery_level)
        print(pm_state)
        print("Getting ready to fly")
        move_with_EEK(scf)
        print(battery_level)
        print(pm_state)
        print(batt_state_ticks)
        print("Flight completed!")
        logconfBatt.stop()       