import re
import time
import queue
# socket is just the end Point the receives data
import socket
import threading
from Adafruit_IO import Client, Feed, RequestError

# Set to your Adafruit IO key
ADAFRUIT_IO_KEY = 'aio_eIys44vlqcLHndzmCP3c1ThYDglF'

# Set to your Adafruit IO username.
ADAFRUIT_IO_USERNAME = 'sara_soso'

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

try: # if we have a location feed
    location = aio.feeds('location')
    print("location exists")

except RequestError: # create a location feed feed
    print("location not exists")
    location= Feed(name="location")
    location = aio.create_feed(location)

try: # if we have a tds feed
    tds = aio.feeds('tds')
    print("tds exists")

except RequestError: # create a tds feed
    print("tds not exist")
    tds = Feed(name ="tds")
    tds =aio.create_feed(tds)

try: # if we have a ph feed
    ph = aio.feeds('ph')
    print("ph exists")

except RequestError: # create a ph feed
    print("ph not exists")
    ph = Feed(name="ph")
    ph = aio.create_feed(ph)
        
try: # if we have an Alert feed
    alert = aio.feeds('alert')
    print("alert exists")

except RequestError: # create an Alert feed
    print("alert not exists")
    alert = Feed(name="alert")
    alert = aio.create_feed(alert)
 
# Receive Data from Client, uisng client ID to process it  
def handle_client(client_socket, client_id , message_queue):
    while True:
        try:
            data = client_socket.recv(1024)
            if not data:
                print(f"Client {client_id} disconnected.")
                break
            print(f"Received from Client {client_id}: {data.decode('utf-8')}")
            message_queue.put((client_id, data.decode('utf-8')))
        except Exception as e:
            print(f"Error handling Client {client_id}: {e}")
            break

    client_socket.close()
value = 0
# Message processing according to Client ID
def process_messages(message_queue):
    while True:
        global value 
        message = message_queue.get()
        if message is None:
            break
        client_id, data = message
        print(f"Processing message from Client {client_id}: {data}")
        # send data to tds feed on adafruit cloud
        if client_id == 'tds_client':
            aio.send(tds.key, data)
        # send data to ph feed on adafruit cloud   
        elif client_id == 'ph_client':
            aio.send(ph.key, data) 
            # if the water is acidic make alert 
            if int(data)<=5:
                aio.send(alert.key,"1" )
            else:
                aio.send(alert.key,"0" )
        # # send data to gps feed on adafruit cloud       
        elif client_id == 'gps_client':  
            # Extract latitude and longitude from the message using RegEx
            lat_match = re.search(r'Latitude=(-?\d+\.\d+)', data)
            lon_match = re.search(r'Longitude=(-?\d+\.\d+)', data)

            if lat_match and lon_match:
                lat = float(lat_match.group(1))
                lon = float(lon_match.group(1))
                ele = 6
                value += 1
                metadata = {'lat': lat, 'lon': lon, 'ele': ele, 'created_at': time.asctime(time.gmtime())}
                aio.send_data(location.key, value, metadata)
                print('\nData Received by Adafruit IO Feed:\n')
            else:
                print(f"Failed to extract latitude and/or longitude from GPS data: {data}")
            
        

def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #  gethostname return the ip address of your network and port number is 1234
    #  choose a specific port to perform your task 
    #  hosting the server on the same machine 
    #  donot use port number less than 4 digits as they are reserved 
    server.bind((socket.gethostname(),1234))
    # prepare server for incomming connections  with a maximum backlog of 5
    server.listen(5)
    print("Server listening on port 1234")
    message_queue = queue.Queue()
    # Start the message processing thread
    processing_thread = threading.Thread(target=process_messages, args=(message_queue,))
    processing_thread.start()
    try:
        while True:
            client_socket, addr = server.accept()
            print(f"Accepted connection from {addr}")
            # Receive client name from the client
            client_id = client_socket.recv(1024).decode('utf-8')
            print(f"Received client name: {client_id}")
            # Start the Client Handler thread
            client_handler = threading.Thread(target=handle_client, args=(client_socket, client_id, message_queue))
            client_handler.start()
 
    except KeyboardInterrupt:
        print("Server terminated.")
    finally:     
        server.close()
        # Stop the message processing thread
        message_queue.put(None)
        processing_thread.join()

if __name__ == "__main__":
    main()
