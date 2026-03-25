# STEMMNet Particle Boron Migration

## Overview
This is the updated STEMMNet code migrated from SparkFun Asset Tracker (ESP32/SARA-R5) to Particle Boron. 

### Key Changes from Original:
1. **Simplified Upload Logic**: Every time data is logged, it's uploaded immediately (no separate upload interval)
2. **Single Configuration Variable**: `DATA_INTERVAL_SECONDS` (replaces LOG_INTERVAL and UPLOAD_INTERVAL)
3. **Default 1 hour interval**: Changed from 5 minutes to 3600 seconds
4. **Webhook-based communication**: Uses Particle webhooks instead of direct HTTP calls
5. **Same data format**: Maintains 111-byte semicolon-delimited format for compatibility

## Hardware Configuration

### Pin Mapping (Particle Boron):
- **D0** (was ONEWIRE_PIN): OneWire/Dallas temperature sensors (5x DS18B20)
- **D3** (was PWM0): MOSFET sensor power control
- **D5** (was D0): OneWire data pin
- **D7**: Built-in LED for status indication
- **A2**: SD card CS (chip select)
- **A4**: I2C SDA (ADS1115 ADCs)
- **A5**: I2C SCL (ADS1115 ADCs)

### Sensor Configuration:
- 5x Dallas DS18B20 temperature sensors (OneWire on D5)
- 2x ADS1115 ADCs at 0x48 and 0x49 (8 analog channels total)
- SD card for data buffering
- Channels 0-3 on ADS0: Moisture sensors
- Channels 0-3 on ADS1: Voltage sensors

## Deployment Instructions

### 1. Flash the Firmware
```bash
# Using Particle CLI
particle flash YOUR_DEVICE_NAME STEMMNet-TB.ino

# Or compile and flash via Particle Web IDE
# Upload STEMMNet-TB.ino to Web IDE and flash
```

### 2. Set Up Webhooks

#### Install via Particle CLI:
```bash
# Navigate to the directory with webhook files
cd /path/to/webhook/files

# Create telemetry webhook
particle webhook create webhook-telemetry.json

# Create config webhook
particle webhook create webhook-config.json

# Verify webhooks are created
particle webhook list
```

#### Or Configure Manually in Particle Console:

**Telemetry Webhook:**
1. Go to Particle Console → Integrations → New Integration → Webhook
2. Settings:
   - Event Name: `tb-telemetry`
   - URL: `http://data.alclimate.com:1000/`
   - Request Type: `POST`
   - Request Format: `Form`
   - Form Fields:
     - `token`: `{{token}}`
     - `id`: `{{id}}`
     - `data`: `{{data}}`
   - Advanced Settings:
     - Enforce SSL: `OFF`
     - Response Topic: `{{PARTICLE_DEVICE_ID}}/hook-response/tb-telemetry`

**Config Webhook:**
1. Go to Particle Console → Integrations → New Integration → Webhook
2. Settings:
   - Event Name: `tb-get-config`
   - URL: `http://data.alclimate.com:1000/config.php`
   - Request Type: `GET`
   - Request Format: `Query Parameters`
   - Query Parameters:
     - `id`: `{{PARTICLE_EVENT_VALUE}}`
   - Advanced Settings:
     - Enforce SSL: `OFF`
     - Response Topic: `{{PARTICLE_DEVICE_ID}}/hook-response/tb-get-config`

### 3. Configure ThingsBoard

Your existing ThingsBoard server at `data.alclimate.com:1000` needs to:

1. **Accept telemetry uploads** at `/` endpoint:
   - Method: POST
   - Form data: `token`, `id`, `data`
   - Data format: semicolon-delimited string (111 bytes)
   - Example: `25.12,-99.99,-99.99,-99.99,-99.99,1234.56,2345.67,3456.78,4567.89,123.45,234.56,345.67,456.78,1704931200;`

2. **Provide configuration** at `/config.php` endpoint:
   - Method: GET
   - Query parameter: `id` (station ID)
   - Response format: CSV with first value being DATA_INTERVAL_SECONDS
   - Example response: `3600,10800,2025,01,09,12,30,00`
   - Only the first value (3600) is used by the new code

### 4. Update Station Configuration

For each station, update the code constants:

```cpp
const char STATION_ID[] = "SN000000";  // Change to your station ID
const char THINGSBOARD_TOKEN[] = "rdbp1d9j7d6ou7bq11t6";  // Your ThingsBoard device token
```

Also update the Dallas sensor addresses in the `dsAddresses[]` array for each physical sensor.

## Data Flow

```
1. Boron wakes up on DATA_INTERVAL_SECONDS
2. Reads all sensors
3. Formats data as semicolon-delimited string
4. Saves to SD card (daily file + upload buffer)
5. Publishes "tb-telemetry" event with data
6. Webhook forwards to data.alclimate.com:1000
7. Webhook response confirms success
8. If successful, clears upload buffer
9. Goes to sleep until next interval
```

## Configuration Updates

To change the data interval remotely:

1. Edit the first value in your ThingsBoard config.php response
2. On next config request (every 24 hours), station will update
3. Or trigger immediate config request by power cycling the device

Valid range: 60 seconds (1 min) to 86400 seconds (24 hours)

## Troubleshooting

### Station not uploading:
```bash
# Check webhook logs in Particle Console
# Verify device is publishing events
particle subscribe tb-telemetry mine

# Check device logs
particle serial monitor
```

### Config not updating:
```bash
# Manually trigger config request
particle call YOUR_DEVICE_NAME "tb-get-config" "SN000000"

# Check webhook response in Console
```

### SD card issues:
- Device will upload single points directly if SD fails
- Check LED patterns: 3 fast blinks = success, 1 long blink = error

### Time sync issues:
- First boot waits 5 minutes for cellular connection
- Subsequent boots wait 1 minute
- Device will use estimated timestamp if Time.isValid() is false

## Monitoring

**LED Status Patterns:**
- Single blink: Logging data
- Triple blink: Success (data logged and uploaded)
- Long blink (500ms): Error

**Serial Debug Output:**
Enable `SERIAL_DEBUG` to see detailed logs via USB serial at 115200 baud.

## Power Consumption

Typical cycle (1 hour interval):
- Wake + sensor reading: ~10 seconds @ 200mA
- Cellular upload: ~20 seconds @ 500mA  
- Deep sleep: 3590 seconds @ 0.1mA
- Average: ~4.5mA

Expected battery life with 10,000mAh:
- ~90 days continuous operation

## Differences from Original Code

| Feature | Original (SARA-R5) | New (Particle) |
|---------|-------------------|----------------|
| Data interval | LOG_INTERVAL (300s) | DATA_INTERVAL (3600s) |
| Upload interval | UPLOAD_INTERVAL (10800s) | Every log cycle |
| Config variable | LOG_INTERVAL_SECONDS, UPLOAD_INTERVAL_SECONDS | DATA_INTERVAL_SECONDS |
| Communication | Direct HTTP | Particle.publish() + webhooks |
| Time sync | SARA-R5 clock | Particle Time.now() |
| Connection mgmt | Manual AT commands | Particle.connect() |

## Migration Checklist

- [ ] Update STATION_ID in code
- [ ] Update THINGSBOARD_TOKEN in code
- [ ] Update dsAddresses[] with actual sensor addresses
- [ ] Flash firmware to Boron
- [ ] Create webhooks in Particle Console
- [ ] Verify ThingsBoard accepts new format
- [ ] Test full cycle with serial monitor
- [ ] Verify SD card logging
- [ ] Verify cellular upload
- [ ] Check ThingsBoard data display
- [ ] Deploy to field location

## Support

For issues or questions:
1. Check serial debug output (115200 baud)
2. Review Particle Console event logs
3. Check webhook integration logs
4. Verify ThingsBoard server logs
