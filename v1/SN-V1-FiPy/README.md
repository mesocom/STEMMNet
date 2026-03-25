# UAH STEMMNet-LD
> Rev. 2022-02-23

## UAH Soil Temperature, Environment, and Moisture Monitoring Network

STEMMNet-LD is the Low-Density / LTE-reliant STEMMNet.

Configuration is relatively simple. After constructing station hardware, follow these simple software steps.

### SIM Card Configuration
- Retrieve a new Hologram SIM card and remove it from all adapters
- Note the SIM number on the front of the card
- Log in to the hologram.io dashboard and select "Activate SIMs"
- Leave all default settings and click "Select Plan"
- Enter the SIM number and "STEMMNet" as the prefix
- Confirm the plan in the final window

### Pycom Board Preparation
- Connect to the Pycom board using a PySense or similar interface
- Open the Pycom Firmware Update utility
  - Click continue until you reach the "Communication" section
  - If using a GPy
    - Download the appropriate firmware (under Pybytes): https://docs.pycom.io/advance/downgrade/
    - Select "Flash from local file" and select the firmware gzip
  - Select "Erase flash file system"
  - Deselect "Enable Pybytes"
  - Click continue ... the board should upgrade

### Initial LTE Radio Configuration
- Insert the SIM card into a new GPy
- Open this project in Atom and activate the Pymakr prompt
- Run the initial_config.py script to allow the board to connect to the LTE network. *It is normal for this to take upwards of 30 minutes ... just let the script run*


### Upload Preparation
- Create a device ID using the last 5 digits of the SIM number and the Hologram Device ID
- Enter the ID in the SIM card name field on the Hologram dashboard
- Generate a random, 20-character, alphanumeric access token
- In Atom, edit the config.json file
  - Insert the appropriate device ID into the "device_id" field
  - Insert the token into the "log_token" field
- Log in to the **emeshaws** server and enter these tokens into the credentials file

### Upload
- In the Pymakr console, select the project folder in the upper-left hand corner (even if it looks like it's already selected)
- Click the "Upload project to device" button on the left side of the console
- Once the upload is complete, allow the GPy to connect to the LTE network and upload a test packet.

## Debugging

LED colors help identify operations on the board. The following progression should occur:

### Boot
1. A brief blue pulse upon inital power-up
2. Solid-burning purple indicating **LTE connection attempt**
  - A red pulse indicates failure. The board will reboot and retry.
3. Solid-burning yellow indicating **clock synchronization**
  - A red pulse indicates failure. The board will reboot and retry.
4. A brief green pulse indicates a successful boot procedure. The board is now operating.

### Logging
- A brief yellow pulse indicating the start of the log process
- A red pulse indicates a failure in the data logging process
- A green pulse after ≈ 30 seconds indicates successful data upload

