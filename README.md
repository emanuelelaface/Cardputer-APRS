# Cardputer ADV APRS over LoRa (EU868) + GNSS

Firmware for **M5Stack Cardputer ADV** that shows GNSS data (grid + satellite sky plot + battery) and sends the current position as an **APRS (TNC2) position packet over LoRa (EU868)**.

## What it does
- Reads GNSS (GPS) and displays:
  - Lat / Lon / Alt / Speed / Date / Time
  - Fix (2D/3D), sats seen/used, fix age
  - Satellite “sky plot”
  - Battery %
- Builds an **APRS TNC2** position string and transmits it with **LoRa EU868**
- LoRa transmit is **non-blocking** (runs in a FreeRTOS task), so UI/GNSS keep updating

## Controls
### Main screen
- **ENTER**: transmit once
- **L**: toggle auto transmit every **5 minutes**
- **C**: open config menu
- **S**: screen on/off

### Config menu
- Type to edit current field
- **DEL** = backspace
- **ENTER** = next field (last field saves and exits)

> Note: in config mode, L/S/C are treated like normal text (they do not trigger actions).

## Configuration (saved on device)
The first boot opens the config menu automatically. You can open it later with **C**.

Fields:
- Callsign (can be empty, but TX will be blocked until set)
- Destcall
- Path
- Symbol table + symbol code
- Comment (optional)

## How to make and load the BIN with M5Burner (no merge/esptool)
This is the exact workflow used for this project: **flash once**, then **export BIN from the device using M5Burner**, and finally **load that BIN whenever you want**.

### A) Create the BIN (Firmware Exporter)
1. Flash the firmware to the Cardputer ADV once (for example from Arduino IDE).
2. Open **M5Burner**
3. Connect the Cardputer ADV via USB
4. Go to **USER CUSTOM**
5. Open **Firmware Exporter**
6. Select the correct serial port and click **Export**
7. Save the exported file (example: `CardputerADVAPRS.bin`)

Now you have a BIN you can reuse.

### B) Load the BIN (Burn)
1. Open **M5Burner**
2. Go to **USER CUSTOM**
3. Choose **Burn** (or “Burn local firmware” depending on your version)
4. Select your exported `CardputerADVAPRS.bin`
5. Select the correct serial port
6. Click **Burn**

That’s it.

## Notes / Legal
- This transmits APRS-like packets over LoRa RF. Make sure you are compliant with your local rules for EU868 and amateur radio.
- If you want the position to appear on APRS-IS/aprs.fi you normally need an IGate receiving your RF and injecting to APRS-IS (this firmware does not talk to APRS-IS directly).
