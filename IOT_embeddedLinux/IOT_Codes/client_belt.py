# socket is just the end Point the receives data
import time
import socket
import requests

token="kPYpB8lTlmrGr_ud9H996gn-5nfF7i5s"

# read virtual pin on blynk app or dashboard
def read(token,pin):
	api_url = "https://blynk.cloud/external/api/get?token="+token+"&"+pin
	response = requests.get(api_url)
	return response.content.decode()

# send data read from blynk to server
def send(client):
    while True:
        data = read(token,'v1')
        client.send(str(data).encode('utf-8'))
        print(data)
        #time.sleep(10)  
 
def main():
    # create instance from socket
    # using socket Family of IPv4   and      TCP 
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # byte streams we send and receive bytes 
    # 1024 is the number of bytes you will receive at a time 
    # or in other words is the buffer size 
    client.connect((socket.gethostname(),1235))
    client_id = 'Blynk_belt_client'
    client.send(client_id.encode('utf-8'))
    try:
        send(client)
    except KeyboardInterrupt:
        print("Blynk Belt Client Terminated.")
    finally:
        client.close()
        
if __name__ == "__main__":
    main()