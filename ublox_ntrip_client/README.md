# ublox_ntrip_client

`ublox_ntrip_client` is a ROS node that retrieves RTCM3 data from an NTRIP caster and outputs it to a Ublox receiver via a serial port.

## Features
- Retrieve RTCM3 data from an NTRIP caster
- Send the retrieved data to a specified serial port

---

## Notes

1. **Internet Connection**
   - An internet connection is required to retrieve data from the NTRIP caster while running the node.

2. **Ublox Device Connection**
   - When sending RTCM3 data to a Ublox device, ensure that a different serial port is used than the one used by the `ublox_gps` node.
   - This means the Ublox device must have two serial ports connected: one for the `ublox_gps` node and another for RTCM3 data transmission.

---

## Usage

### 1. Set Environment Variables
Before running the node, set the following environment variables:

```bash
export NRTRIP_CASTER_USERNAME=<username>
export NRTRIP_CASTER_PASSWORD=<password>
export NRTRIP_CASTER_ADDRESS=<ntrip_caster_ip_address>
export NRTRIP_CASTER_PORT=2101
export NRTRIP_CASTER_MOUNTPOINT=<mountpoint>
export RTCM3_SERIAL_OUT=ttyUSB0
```

#### Environment Variables Description

| Environment Variable         | Description                          | Example                     |
|------------------------------|--------------------------------------|-----------------------------|
| `NRTRIP_CASTER_USERNAME`     | NTRIP caster username                | `username`                  |
| `NRTRIP_CASTER_PASSWORD`     | NTRIP caster password                | `password`                  |
| `NRTRIP_CASTER_ADDRESS`      | NTRIP caster IP address              | `ntrip.ales-corp.co.jp`     |
| `NRTRIP_CASTER_PORT`         | NTRIP caster port number             | `2101`                      |
| `NRTRIP_CASTER_MOUNTPOINT`   | NTRIP caster mountpoint              | `32M7NHS`                   |
| `RTCM3_SERIAL_OUT`           | Serial port for outputting RTCM3 data| `ttyUSB0`                   |

### 2. Launch the Node
After setting the environment variables, run the following command to launch the node using the `launch` file:

```bash
roslaunch ublox_ntrip_client ntrip_client.launch
```

---
