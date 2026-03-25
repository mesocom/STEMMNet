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

import json
import utime
import binascii

from machine import I2C, Pin, RTC, Timer, reset
from network import LTE, WLAN
import pycom

import ads1x15
from onewire import DS18X20
from onewire import OneWire





class STEMMNET():

    ##########################################
    # Initialize station
    ##########################################
    def __init__(self):

        """
        STEMMNet Station initialization procedure
        * Read config config file
        * Initialize ADC
        * Initialize OneWire interface
        * Initialize human interfaces
        * Set RTC time
        """
        
        # Turn off WiFi radio
        wlan = WLAN()
        wlan.deinit()

        # Read configuration file
        config_file = open("config.json", "r")
        self.cfg = json.loads(config_file.read())
        config_file.close()
        self.config_mode = False

        # Set up MOSTFET pin
        self.mosfet_pin = Pin(self.cfg["mosfet_pin"], mode = Pin.OUT)

        # Set up ADC
        self.i2c = I2C()
        self.adc = ads1x15.ADS1115(self.i2c, 72, 1)  # 0x72, gain=1 (±4.096V)

        # Initialize One Wire interface
        self.one_wire = OneWire(Pin(self.cfg["onewire_pin"]))
        self.temp_sensors = DS18X20(self.one_wire)

        # Set up human interfaces
        self.config_switch = Pin(self.cfg["config_pin"], mode = Pin.IN, pull = Pin.PULL_DOWN)  # config switch, pulled down externally
        self.config_switch.callback(Pin.IRQ_RISING, handler = self.configure_temp_sensors)  # trigger config on rising edge of switch activation
        pycom.heartbeat(False)  # stop LED heartbeat
        
        # Reset cell modem
        lte = LTE()
        lte.reset()

        # Connect to cell network
        self.led_status("connecting")
        print("connecting")
        try:
            self.lte_connect()
        except:
            self.reboot()
        
        self.led_status("standby")
        print("connected")

        # Set RTC time
        self.rtc = RTC()  # initialize synchronization
        self.sync_rtc()
        
        self.lte_disconnect()
            
        self.led_status("success")
        print("init complete")


    ##########################################
    # Reboot the board
    ##########################################
    def reboot(self):
        
        """
        Reboots the board, accounting for LTE retry spamming issues
        """

        self.lte_disconnect()
        self.led_status("reset")
        utime.sleep_ms(60000)
        self.led_status("off")
        reset()


    ##########################################
    # Synchronize RTC
    ##########################################
    def sync_rtc(self):
        
        """
        Synchronizes the RTC to Google's NTP server
        """
        
        try:
            self.rtc.ntp_sync("time.google.com", update_period = 60000, backup_server = None)
            
            while not self.rtc.synced():
                utime.sleep_ms(500)
        except:
            self.reboot()
        
        return True


    ##########################################
    # LTE Connection
    ##########################################
    def lte_connect(self):

        """
        Safely initialize and connect the LTE modem to the network
        """

        # Initialize LTE modem
        self.lte = LTE()

        # Attach to network
        self.lte.attach()
        while not self.lte.isattached():
            utime.sleep_ms(250)

        # Start a data session
        self.lte.connect(cid = 3)
        while not self.lte.isconnected():
            utime.sleep_ms(250)

        return True



    ##########################################
    # LTE Disconnect
    ##########################################
    def lte_disconnect(self):

        """
        Safely disconnect the LTE radio from the network and reboot it
        """

        self.lte.disconnect()
        self.lte.deinit(detach = True, reset = True)

        return True



    ##########################################
    # LED status
    ##########################################
    def led_status(self, status = "off"):

        """
        Blink the onboard LED to display a status
        * Blinks with a delay for error and success modes
        * Solid burn for standby, config, and off
        """

        colors = {
            "off" : 0x000000,  # off
            "connecting" : 0xff00ff, # purple connecting
            "standby" : 0xffd700, # yellow standby (system operations)
            "config" : 0x0000ff,  # blue config
            "error" : 0xff0000,  # red error
            "success" : 0x00ff00,  # green success
            "reset" : 0xffffff # white resetting
        }

        # Catch bad color
        if status in colors.keys():
            color = colors[status]
        else:
            color = 0x000000

        # Change LED color
        pycom.rgbled(color)

        # Blink success and error LEDs
        if status == "success" or status == "error":
            utime.sleep_ms(1000)
            pycom.rgbled(colors["off"])

        return True



    ##########################################
    # Temperature sensor configuration mode
    ##########################################
    def configure_temp_sensors(self, switch):

        """
        Configure temperature sensor order by sequentially reading new IDs
        * Will flash an error LED if sensors are already attached
        * Flashes a success blink when new sensors are attached
        * Stops when configuration switch is deactivated or 3 sensors have been attached
        """
        
        print("config mode")

        # Prevents multiple calls due to debounce
        if self.config_mode == True:
            return True

        # Prevents station from logging while sensors are being configured
        self.config_mode = True

        # Power on MOSFET and wait 5 seconds for sensors to boot
        self.led_status("standby")
        self.mosfet_pin(1)
        utime.sleep_ms(5000)
        self.led_status("off")

        num_sensors = 0
        new_sensors = []

        # Check for sensors still attached
        #[rom for rom in self.one_wire.scan() if rom[0] == 0x10 or rom[0] == 0x28]
        while len(self.one_wire.scan()) > 0 and self.config_switch() == 1:
            self.led_status("error")
        self.led_status("off")

        # Wait for new sensors
        while self.config_switch() == 1 and num_sensors < 3:

            self.led_status("config")  # turn on blue config LED

            # Scan for sensors
            raw_sensors = self.one_wire.scan()
            sensors = [binascii.hexlify(s).decode() for s in raw_sensors]
            utime.sleep_ms(500)

            # Add new sensors if any are available
            if len(sensors) > num_sensors:
                num_sensors = num_sensors + 1
                new = [s for s in sensors if s not in new_sensors][0]
                new_sensors.append(new)
                self.led_status("success")

        # Write new sensors to config file
        if num_sensors > 0:
            self.cfg["temp_sensors"] = new_sensors
            with open("config.json", "w") as config_file:
                config_file.write(str(json.dumps(self.cfg)))
                config_file.close()

        # Power off MOSFET
        self.mosfet_pin(0)

        # Ensure LED is off
        self.led_status("off")
        
        self.config_mode = False

        return True



    ##########################################
    # Read DS18B20 sensors over One Wire
    ##########################################
    def read_temperatures(self):

        """
        Read DS18B20 temperatures
        * Returns in the order stored in the config file
        * Values are returned as an array of strings in the format dd.dd (ºC)
        * -998 indicates missing sensors, -999 indicates failed reads
        """

        sensors = [binascii.unhexlify(s) for s in self.cfg["temp_sensors"]]

        # Start temperature conversion on sensors
        for sensor in sensors:
            self.temp_sensors.start_conversion(rom = sensor)
            utime.sleep_ms(2000)

        # Read sensors, if any
        temps = []
        for sensor in sensors:
            try:
                temps.append(self.temp_sensors.read_temp_async(rom = sensor))
            except:
                temps.append(-999)

        # Fill missing / erroneous sensors with hardware error value
        for i in range(0,3-len(temps)):
            temps.append(-998)
        if None in temps:
            temps[temps == None] = -999

        return temps



    ##########################################
    # Read external ADC to get moisture sensor voltages
    ##########################################
    def read_moistures(self):

        """
        * Reads ch0, ch1, ch2
        * Converts raw voltage to whole millivolts
        * Returns -998 if there is an ADC error
        """

        try:
            raw_voltages = [self.adc.raw_to_v(v) * 1000 for v in [self.adc.read(0, c) for c in range(0,3)]]
            voltages = raw_voltages  #[v if v > 900 and v < 2700 else -999 for v in raw_voltages]
        except:
            voltages = [-998,-998,-998]

        return voltages



    ##########################################
    # Construct data string
    ##########################################
    def get_current_data(self, current_dt):

        """
        Collects current sensor data and packages
        values with the current UTC time since epoch
        """

        # Power on MOSFET and wait 5 seconds to record data
        self.mosfet_pin(1)
        utime.sleep_ms(5000)

        # Read sensor values
        temps = [str(t) for t in self.read_temperatures()]  # read temperatures as strings
        moistures = [str(m) for m in self.read_moistures()]  # read moistures as strings

        # Join time and values
        log_data = "stemmnet,sensor_id={0} td={1},tm={2},ts={3},md={4},mm={5},ms={6} {7}\n".format(
        self.cfg["device_id"], temps[0], temps[1], temps[2], moistures[0], moistures[1], moistures[2], current_dt)

        # Power off MOSFET
        self.mosfet_pin(0)

        return log_data


# EOF
