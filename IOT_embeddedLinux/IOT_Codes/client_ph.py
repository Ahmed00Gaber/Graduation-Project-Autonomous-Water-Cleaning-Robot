# PH Client collect data for monitoring the Water Quality
# Send data to server, Then server send it to Adafruit Cloud Platform
import socket
# socket is just the end Point the receives data
import time
import busio
import board
import digitalio
import numpy as np
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the CS (chip select)
cs = digitalio.DigitalInOut(board.D5)

# Create the MCP object
mcp = MCP.MCP3008(spi, cs)

AnalogIn.VREF = 5.0
# Create an analog input channel on pin1
ph_chan = AnalogIn(mcp, MCP.P1)


# map function to map from [0:65536] digital values of adc to [0 :14] ph values
def map_range(value, from_range, to_range):
    return np.interp(value, [from_range[0], from_range[1]], [to_range[0], to_range[1]])
# send ph data to server
def send_ph_data(client_socket):
    while True:
        print("Raw ADC Value:", ph_chan.value)
        print("ADC Voltage: {:.2f}V".format(ph_chan.voltage))
        ph_value = map_range(ph_chan.value, [0, 65536], [0, 14])
        print(ph_value)
        client_socket.send(str(ph_value).encode('utf-8'))
        time.sleep(30)

def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # byte streams we send and receive bytes 
    # 1024 is the number of bytes you will receive at a time 
    # or in other words is the buffer size 
    client.connect((socket.gethostname(),1234))
    client_id = 'ph_client'
    client.send(client_id.encode('utf-8'))
    try:
        send_ph_data(client)
    except KeyboardInterrupt:
        print("ph Client Terminated.")
    finally:
        client.close()
        
if __name__ == "__main__":
    main()
