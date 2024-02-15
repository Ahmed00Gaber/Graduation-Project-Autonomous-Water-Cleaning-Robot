import queue
# socket is just the end Point the receives data
import socket
import requests
import threading
import RPi.GPIO as GPIO

Belt_pin = 16
pump_pin = 18

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

GPIO.setup(Belt_pin, GPIO.OUT, initial=GPIO.LOW)     # Set physical pin 16 to be an output pin and set initial value to low (off)
GPIO.setup(pump_pin, GPIO.OUT, initial=GPIO.LOW)   # Set physical pin 18 to be an output pin and set initial value to low (off)

token="kPYpB8lTlmrGr_ud9H996gn-5nfF7i5s"

# Write Data to blynk app that read from Ada Fruit cloud dashboard
def write(token,pin,value):
	api_url = "https://blynk.cloud/external/api/update?token="+token+"&"+pin+"="+value
	response = requests.get(api_url)

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
  
# Message processing according to Client ID
def process_messages(message_queue):
    while True:
       
        message = message_queue.get()
        if message is None:
            break
        client_id, data = message
        print(f"Processing message from Client {client_id}: {data}")
        # turn on, off the alert led
        if client_id == 'client_Ada':
            print(data)
            write(token,"v0",data)
        # turn on, off belt according to app and user     
        elif client_id == 'Blynk_belt_client':
            print(data)
            if data == '1':
                GPIO.output(Belt_pin, GPIO.HIGH)  # Turn on
            else:
                GPIO.output(Belt_pin, GPIO.LOW) 
        # turn on, off belt according to app and user        
        elif client_id == 'Blynk_pump_client':
            print(data)
            if data == '1':
                GPIO.output(pump_pin, GPIO.HIGH) # Turn on
            else:
                GPIO.output(pump_pin, GPIO.LOW) # turn off
            

def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #  gethostname return the ip address of your network and port number is 1235
    #  choose a specific port to perform your task 
    #  hosting the server on the same machine 
    #  donot use port number less than 4 digits as they are reserved 
    server.bind((socket.gethostname(),1235))
    # prepare server for incomming connections  with a maximum backlog of 5
    server.listen(5)
    print("Server listening on port 1235")
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
