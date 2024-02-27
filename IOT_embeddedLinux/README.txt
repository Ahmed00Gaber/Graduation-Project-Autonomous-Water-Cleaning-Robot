# Description
## Server Initialization:**
   - The `server_ada.py` server for Adafruit Cloud integration will be initiated.
   - Wait for approximately 10 seconds to ensure that the server is ready to listen to clients.

## Client Execution:**
   - After the server is ready, the following clients will automatically run in separate terminals:
     - TDS Client: `python3 client_tds.py`
     - pH Client: `python3 client_ph.py`
     - GPS Client: `python3 client_gps.py`

## Adafruit Cloud Integration:**
   - The server will forward data received from clients to the Adafruit Cloud.

## File Descriptions:**

   - `server_ada.py`: Server script for Adafruit Cloud integration.
   - `client_tds.py`: Client script for TDS data.
   - `client_ph.py`: Client script for pH data.
   - `client_gps.py`: Client script for GPS data.

   These clients send data to the server, which interprets the signals and controls the devices accordingly
