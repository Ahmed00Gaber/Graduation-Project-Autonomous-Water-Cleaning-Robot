# socket is just the end Point the receives data
import time
import socket
from Adafruit_IO import Client, Feed, RequestError

# Set to your Adafruit IO key
ADAFRUIT_IO_KEY = 'aio_eIys44vlqcLHndzmCP3c1ThYDglF'

# Set to your Adafruit IO username.
ADAFRUIT_IO_USERNAME = 'sara_soso'

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

try:  # if we have an alert feed
    alert = aio.feeds('alert')
    print("alert exists")

except RequestError: # create an alert feed
    print("alert not exists")

def read_Ada_data(client_socket):  
    while True:
        # Receive data from Adafruit could alert feed
        data = aio.receive(alert.key)
        print(data)
        value = data.value
        # Send only the 'value' over the socket
        client_socket.send(str(value).encode('utf-8'))
        time.sleep(10)       

def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # byte streams we send and receive bytes 
    # 1024 is the number of bytes you will receive at a time 
    # or in other words is the buffer size 
    client.connect((socket.gethostname(),1235))
    client_id = 'client_Ada'
    client.send(client_id.encode('utf-8'))
    try:
        read_Ada_data(client)
    except KeyboardInterrupt:
        print("Read_ADA Client Terminated.")
    finally:
        client.close()
        
if __name__ == "__main__":
    main()
