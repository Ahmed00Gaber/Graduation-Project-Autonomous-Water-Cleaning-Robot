# TDS Client collect data for monitoring the Water Quality
# Send data to server, Then server send it to Adafruit Cloud Platform
import socket
# socket is just the end Point the receives data
import time
import busio
import board
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the CS (chip select)
cs = digitalio.DigitalInOut(board.D5)

# Create the MCP object
mcp = MCP.MCP3008(spi, cs)

AnalogIn.VREF = 5.0
# Create an analog input channel on pin 0
TDS_chan = AnalogIn(mcp, MCP.P0)

temperature = 25

# equaitions, parameters according to sensor datasheet
def send_TDS_data(client_socket):
    while True:
        print("Raw ADC Value:", TDS_chan.value)
        print("ADC Voltage: {:.2f}V".format(TDS_chan.voltage))
        TDS_voltage = f"{TDS_chan.voltage}"
        compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0)
        compensationVolatge = float(TDS_voltage) / compensationCoefficient
        TDS_data = (133.42 * compensationVolatge**3 - 255.86 * compensationVolatge**2 + 857.39 * compensationVolatge) * 0.5
        print(f"TDS value is {TDS_data} ppm")
        client_socket.send(str(TDS_data).encode('utf-8'))
        time.sleep(60)



def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # byte streams we send and receive bytes 
    # 1024 is the number of bytes you will receive at a time 
    # or in other words is the buffer size 
    client.connect((socket.gethostname(),1234))
    client_id = 'tds_client'
    client.send(client_id.encode('utf-8'))
    try:
        send_TDS_data(client)
    except KeyboardInterrupt:
        print("TDS Client Terminated.")
    finally:
        client.close()
        
if __name__ == "__main__":
    main()
