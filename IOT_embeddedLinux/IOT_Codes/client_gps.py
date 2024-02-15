# GPS Client collect data for monitoring the bot location
# Send data to server, Then server send it to Adafruit Cloud Platform
import socket
# socket is just the end Point the receives data
import time
import serial
import pynmea2
port = "/dev/ttyAMA0"

def send_gps_data(client_socket):
    with serial.Serial(port, baudrate=9600, timeout=0.5) as ser:
        while True:
            newdata = ser.readline()
            print(newdata)
            if newdata.startswith(b"$GPRMC"):
                try:
                    newmsg = pynmea2.parse(newdata.decode('utf-8', errors='ignore'))
                    lat = newmsg.latitude
                    lng = newmsg.longitude
                    gps = f"Latitude={lat} and Longitude={lng}"
                    print(gps)
                    client_socket.send(gps.encode('utf-8'))
                    time.sleep(60)
                except pynmea2.nmea.ParseError:
                    print("Invalid data")    

def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # byte streams we send and receive bytes 
    # 1024 is the number of bytes you will receive at a time 
    # or in other words is the buffer size 
    client.connect((socket.gethostname(),1234))
    client_id = 'gps_client'
    client.send(client_id.encode('utf-8'))
    try:
        send_gps_data(client)
    except KeyboardInterrupt:
        print("GPS Client Terminated.")
    finally:
        client.close()
        
if __name__ == "__main__":
    main()
