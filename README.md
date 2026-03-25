# STEMMNet

STEMMNet is a modular environmental telemetry and monitoring network utilizing Particle Boron (cellular) hardware and various sensor arrays (SDI-12, Analog, Digital).

## Repository Structure

The project is organized into versions, with the current focus on **v2** (Stable/Cellular):

* **[v2/SN-PB-SDI12](v2/SN-PB-SDI12)**: 
    * Main station firmware for Particle Boron.
    * Supports up to 10 SDI-12 sensors (addresses '0'-'9').
    * Direct HTTP POST ingestion to custom PHP backend (`api.alclimate.com`).
    * Local MicroSD queueing and daily log file rotation.
* **[v2/SN2-PB-Analog](v2/SN2-PB-Analog)**:
    * Analog/Temperature focused firmware.
    * Support for dual ADS1115 (8 total analog channels) and DS18B20 temperature sensor arrays (OneWire).
    * Utilizes identical direct-upload architecture to the SDI-12 version for unified backend processing.
* **[v2/SN2-PB-SDI12-Terminal](v2/SN2-PB-SDI12-Terminal)**: 
    * Utility firmware for onsite SDI-12 sensor testing and manual address configuration.

## Archive (v1)

These folders contain legacy versions of the STEMMNet project for archival purposes:

* **[v1/SN-V1-AssetTracker](v1/SN-V1-AssetTracker)**: Original STEMMNet prototype based on the SparkFun Asset Tracker (legacy STM32).
* **[v1/SN-V1-FiPy](v1/SN-V1-FiPy)**: Experimental version utilizing the Pycom FiPy (ESP32) platform.

## Key Features

* **Direct Uploads**: Communicates directly with custom PHP ingestion servers via TCP/IP (plaintext port 80 to minimize cellular data overhead).
* **Power Management**: Aggressive sleep cycles and sensor-bus power control for year-round remote operation on solar/lithium power.
* **Redundancy**: All readings are logged to a local MicroSD card in daily `.txt` files; if the cellular signal fails, records are queued in a buffer and uploaded automatically during the next successful connection.

## Configuration

Stations are configured over-the-air via the `/ingest/config.php` endpoint. 
* Common parameters include `LOG_INTERVAL_SECONDS` (pushed once per day or upon initial boot).

## Hardware

Designed for the **Particle Boron** (LTE/2G/3G) development board. 
Compatible with various SDI-12 environmental sensors (e.g., Meter Group, Acclima, Campbell Scientific) and standard analog transducers.

---

## License

This software is released under a **Non-Commercial / Academic Use License**. 

* **Permitted**: Use for hobbyist, personal, classroom, and academic research purposes.
* **Restricted**: Commercial use, resale, or embedding into commercial products is strictly prohibited without explicit written consent from the author.
