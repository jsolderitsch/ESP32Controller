from DroneBlocksTelloSimulator.DroneBlocksSimulatorContextManager import DroneBlocksSimulatorContextManager
import socket
import time

# replace with IP of the host running Python
UDP_IP = "192.168.1.199"
# UDP_IP = "192.168.1.14"
UDP_PORT = 8889

def move_with_Controller(sim_key):
    distance = 40
    with DroneBlocksSimulatorContextManager(simulator_key=sim_key) as drone:
        while True:
        # Listen for commands
            data, addr = sock.recvfrom(1024)
            decodedData = data.decode('utf-8').rstrip('\x00')
            if '_v' not in decodedData:
                print(f"Received packet from {addr}: {decodedData}")        
            packetPieces = decodedData.split(' ')
            if 'takeoff' in decodedData:
                drone.takeoff()
                time.sleep(0.5)
                sock.sendto("takeoff ok".encode('utf-8'), addr)
            elif 'land' in decodedData:
                drone.land()
                time.sleep(0.5)
                sock.sendto("land ok".encode('utf-8'), addr)
            elif 'up' in decodedData:
                drone.fly_up(distance, 'cm')
                time.sleep(0.5)
                sock.sendto("up ok".encode('utf-8'), addr)
            elif 'down' in decodedData:
                drone.fly_down(distance, 'cm')
                time.sleep(0.5)
                sock.sendto("down ok".encode('utf-8'), addr)
            elif 'forward' in decodedData:
                drone.fly_forward(distance, 'cm')
                time.sleep(0.5)
                sock.sendto("forward ok".encode('utf-8'), addr)
            elif 'back' in decodedData:
                drone.fly_backward(distance, 'cm')
                time.sleep(0.5)
                sock.sendto("back ok".encode('utf-8'), addr)
            elif 'right' in decodedData:
                drone.fly_right(distance, 'cm')
                time.sleep(0.5)
                sock.sendto("right ok".encode('utf-8'), addr)
            elif 'left' in decodedData:
                drone.fly_left(distance, 'cm')
                time.sleep(0.5)
                sock.sendto("left ok".encode('utf-8'), addr)
            elif 'ccw' in decodedData:
                drone.yaw_left(90)
                time.sleep(0.5)
                sock.sendto("ccw ok".encode('utf-8'), addr)
            elif 'cw' in decodedData:
                drone.yaw_right(90)
                time.sleep(0.5)
                sock.sendto("cw ok".encode('utf-8'), addr)
            elif 'flip' in decodedData:
                drone.flip_backward()
                time.sleep(0.5)
                sock.sendto("flip ok".encode('utf-8'), addr)
            elif 'go' in decodedData:
                drone.fly_to_xyz(int(packetPieces[1]),int(packetPieces[2]),0,'cm')
                time.sleep(0.5)
                sock.sendto("go ok".encode('utf-8'), addr)
            elif 'curve' in decodedData:
                drone.fly_curve(int(packetPieces[1]),
                                 int(packetPieces[2]),
                                 int(packetPieces[3]),
                                 int(packetPieces[4]),
                                 int(packetPieces[5]),
                                 int(packetPieces[6]),'cm')
                time.sleep(0.5)
                sock.sendto("curve ok".encode('utf-8'), addr)
            elif 'emergency' in decodedData:
                drone.land()
                time.sleep(0.5)
                sock.sendto("Kill!".encode('utf-8'), addr)
                break


if __name__ == '__main__':
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}")

    # Replace with your DroneBlocks Simulator Key
    sim_key = '0bcf4645-4c65-4907-a971-c9d05e8bef70'
    move_with_Controller(sim_key)
