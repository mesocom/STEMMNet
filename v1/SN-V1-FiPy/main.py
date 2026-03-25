##########################################
#
#              UAH STEMMNet
#   Soil Temperature, Environment, and
#      Moisture Monitoring Network
#
#      Written by Nick Perlaky © 2022
#
#   MicroPython script for a Pycom FiPy
#
##########################################

# Imports
import urequests
import stemmnet
import utime
import uos
from machine import WDT, reset

buffer_file_path = "/flash/buffer.log"

# Create watchdog timer with 300 second timeout
wdt = WDT(timeout = 120000)


# Set up station
SN = stemmnet.STEMMNET()


# Check for data directory
if "data" not in uos.listdir("/flash"):
    uos.mkdir("/flash/data")


def log_data():
    
    # Standby blink to indicate start of log process
    SN.led_status("standby")
    utime.sleep_ms(500)
    SN.led_status("off")
    utime.sleep_ms(500)

    # Get current data
    current_data = SN.get_current_data(current_timestamp)
    
    # Log point to temporary buffer
    buffer_file = open(buffer_file_path, "a+")
    buffer_file.write(current_data)
    buffer_file.close()
    
    SN.led_status("success")
    print("logged")
    
    return None
    
    
def upload_data():
    
    # Open network connection
    try:
        SN.lte_connect()
    except:
        SN.lte.disconnect()
        return False

    # Read data buffer
    buffer_file = open(buffer_file_path, "r")
    data_buffer = buffer_file.read()
    buffer_file.close()

    # Construct and send POST request to log remotely
    response = urequests.post(
        SN.cfg["log_url"],
        headers = {"Authorization":"Token " + SN.cfg["api_token"], "Content-Type":"text/plain; charset=utf-8", "Accept":"application/json"},
        data = bytes(data_buffer, 'utf-8')
        )

    if not response:
        SN.led_status("error")
        SN.lte_disconnect()
        return False
    
    # Close network connection
    try:
        SN.lte_disconnect()
    except:
        SN.led_status("error")
        reset()
        return False
    
    # Clear temporary buffer
    buffer_file = open(buffer_file_path, "w")
    buffer_file.write("")
    buffer_file.close()

    # Blink succes
    SN.led_status("success")
    print("uploaded")
    
    return None


# Run data logger continuously
while True:

    # Feed watchdog timer
    wdt.feed()

    # Get current time
    current_timestamp = utime.time()

    if SN.config_mode == True:
        utime.sleep_ms(250)
        continue

    # Catch log time
    if current_timestamp % int(SN.cfg["log_interval"]) == 0:
    
        print("logging")
        try:
            log_data()
        except:
            continue
        
        if current_timestamp % int(SN.cfg["upload_interval"]) == 0:
            print("Uploading...")
            try:
                upload_data()
            except:
                print("Upload failure.")
                continue
        
        if current_timestamp % int(SN.cfg["rtc_sync_interval"]) == 0:
            try:
                SN.sync_rtc()
                SN.lte.factory_reset()
            except:
                SN.lte.factory_reset()
                continue


# EOF
