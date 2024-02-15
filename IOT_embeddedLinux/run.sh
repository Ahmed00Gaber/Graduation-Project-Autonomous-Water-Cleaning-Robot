#!/bin/bash

# Change to the directory containing your Python files
cd /home/sarabarbar25/lab_project

# Run the server scripts in the background
nohup lxterminal -e python3 server_ada.py &
nohup lxterminal -e python3 server_blynk.py &

# Sleep for a while to allow the server scripts to start
sleep 10

# Run the client scripts in the background
nohup lxterminal -e python3 client_tds.py &
nohup lxterminal -e python3 client_ph.py &
nohup lxterminal -e python3 client_gps.py &

# Run additional client scripts
nohup lxterminal -e python3 client_ada.py &
nohup lxterminal -e python3 client_pump.py &
nohup lxterminal -e python3 client_belt.py