# CAN Module User Manual

Script Path: `agx_arm_ros/scripts`

**Note**: The CAN module here only supports the CAN module that comes with the robotic arm; other CAN modules are not supported.

Install CAN Tools

```shell
sudo apt update && sudo apt install can-utils ethtool
```

These two tools are used to configure the CAN module.

When only a single CAN module is connected to the computer, you can **quickly activate** it by following these steps:

Open a terminal window and execute the following commands in sequence:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts 
bash can_activate.sh
```

If you see`ip: command not found` when executing a bash script, install the `ip` command, typically with: 

`sudo apt-get install iproute2`

## 1 Find CAN Modules

Run:

```bash
bash find_all_can_port.sh
```

After entering your password, if the CAN module is plugged into the computer and detected, the output will look like this:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

If multiple CAN modules are present, the output will look like this:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

There will be one line like `Interface can1 is connected to USB port 3-1.1:1.0` for each CAN module detected.

- `can1` is the name of the CAN module found by the system.
- `3-1.1:1.0` is the USB port the CAN module is connected to.

If a CAN module was previously activated with a different name (e.g., `can_piper`), the output will be:

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

If no CAN module is detected, only the following will be printed:

```bash
Both ethtool and can-utils are installed.
```

## 2  Activate a Single CAN Module (using `can_activate.sh`)

### (1) Find the USB port hardware address of the CAN module

Unplug all CAN modules, then plug only the CAN module connected to the robotic arm into the PC. Run:

```shell
bash find_all_can_port.sh
```

Record the `USB port` value, e.g., `3-1.4:1.0`.

### (2) Activate the CAN device

Assuming the `USB port` value is `3-1.4:1.0`, run:

```bash
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

The CAN device plugged into the USB port with hardware ID `3-1.4:1.0` is renamed to `can_piper`, set to a baud rate of 1,000,000, and activated.

### (3) Verify activation

Run `ifconfig` and check if `can_piper` appears. If it does, the CAN module is set up successfully.

### (4) Special note

If only one CAN module is plugged into the computer, you can run directly:

```bash
bash can_activate.sh can0 1000000
```

Here, `can0` can be replaced with any name, and `1000000` is the baud rate.

## 

## 3 Activate Multiple CAN Modules Simultaneously (using `can_muti_activate.sh`)

First, check how many official CAN modules are plugged into the computer (we assume 2 in this example).

**Note**: If 5 CAN modules are plugged in, you can activate only the specified ones.

### 3.1 Record the USB port hardware address for each CAN module

Plug and unplug CAN modules one by one, and record the USB port hardware address for each module.

In `can_muti_activate.sh`, the number of elements in the `USB_PORTS` parameter equals the number of CAN modules to be pre-activated (we assume 2 here).

#### (1) Plug in the first CAN module alone

Run:

```shell
bash find_all_can_port.sh
```

Record the `USB port` value, e.g., `3-1.4:1.0`.

#### (2) Plug in the next CAN module

**Do not** use the same USB port as the previous one. Run:

```shell
bash find_all_can_port.sh
```

Record the `USB port` value for the second CAN module, e.g., `3-1.1:1.0`.

**Note**:

- If no CAN modules have been activated before, the first one plugged in defaults to `can0`, the second to `can1`.
- If they were activated before, they retain their previously assigned names.

### 3.2 Predefine USB ports, target interface names, and baud rates

Assuming the recorded `USB port` values are `3-1.4:1.0` and `3-1.1:1.0`, replace the values inside the double quotes in `USB_PORTS["1-9:1.0"]="can_left:1000000"` with these addresses.

The final result will be:

```bash
USB_PORTS["3-1.4:1.0"]="can_left:1000000"
USB_PORTS["3-1.1:1.0"]="can_right:1000000"
```

**Explanation**:

- The CAN device on USB port `3-1.4:1.0` is renamed to `can_left` with a baud rate of 1,000,000 and activated.
- The CAN device on USB port `3-1.1:1.0` is renamed to `can_right` with a baud rate of 1,000,000 and activated.

### 3.3 Activate multiple CAN modules

Run:

```
bash can_muti_activate.sh
```

### 3.4 Verify multiple CAN module setup

Run `ifconfig` and check if both `can_left` and `can_right` have CAN data.
