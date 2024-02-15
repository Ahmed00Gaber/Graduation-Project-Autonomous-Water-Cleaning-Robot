# Lab Project README

## Introduction

This project consists of a system with multiple components, including servers and clients, to monitor and control various devices.

## Getting Started

1. **Run the System:**
   - Open a terminal and navigate to the project directory.
   - Execute the following command to run the system:
     ```bash
     ./run.sh
     ```
   - This script will start the necessary servers and clients.

2. **Server Initialization:**
   - Two servers will be initiated (`server_ada.py` for Adafruit Cloud integration and `server_blynk.py` for Blynk mobile app integration).
   - Wait for approximately 10 seconds to ensure that the servers are ready to listen to clients.

3. **Client Execution:**
   - After the servers are ready, the following clients will automatically run in separate terminals:
     - TDS Client: `python3 client_tds.py`
     - pH Client: `python3 client_ph.py`
     - GPS Client: `python3 client_gps.py`
     - Adafruit Client: `python3 client_ada.py`
     - Pump Client: `python3 client_pump.py`
     - Belt Client: `python3 client_belt.py`

4. **Adafruit Cloud Integration:**
   - The servers will forward data received from clients to the Adafruit Cloud.

5. **Coordinator:**
   - coordinator to get alerts from the Adafruit Cloud and send them to the server.

6. **Mobile App Clients:**
   - Two clients (`client_pump.py` and `client_belt.py`) read the mobile app for pump and conveyor belt status.
   - These clients send data to the server, which interprets the signal and controls the devices accordingly.

## File Descriptions

- `server_ada.py`: Server script for Adafruit Cloud integration.
- `server_blynk.py`: Clients send data to the server, which interprets the signal and controls the devices accordingly.
- `client_tds.py`: Client script for TDS data.
- `client_ph.py`: Client script for pH data.
- `client_gps.py`: Client script for GPS data.
- `client_ada.py`: Client script for reading data from Adafruit Cloud and send it to server.
- `client_pump.py`: Client script for reading pump status from the mobile app.
- `client_belt.py`: Client script for reading conveyor belt status from the mobile app.

