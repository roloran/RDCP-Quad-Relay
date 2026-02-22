# ROLORAN RDCP Two-Channel LoRa Relay

This repository holds the ESP32 firmware implementation for two-channel RDCP v0.4 relays according to the [RDCP Specs](https://github.com/roloran/RDCP-Specs) as developed in the [ROLORAN project](https://dtecbw.de/home/forschung/unibw-m/projekt-roloran) for the Neuhaus scenario v2.0.

RDCP v0.4 relays use two LoRa channels (in the 433 MHz and the 868 MHz radio frequency bands) to facilitate the scenario-wide bidirectional communication between headquarters and [mobile devices](https://github.com/roloran/ROLODECK) in blackout crisis situations. RDCP v0.4 relays use an ESP32 MCU on a custom PCB along with two EBYTE-based LoRa radios. They are intended to be integrated with RDCP DAs ("Digitale Anschlagtafeln", interactive digital bulletin boards for official announcements) but can be used on their own as well (thus serving as BBKs, backbone nodes).

## Compilation and Flashing of the Two-Channel Relay

This repository uses PlatformIO (PIO), and thus all dependency management and workflows should be handled automatically or at least similar to other PIO projects.

While using PIO through IDEs like Visual Studio Code may work if you prefer GUIs, the recommended workflow is CLI-based along with [ROLORAN-Terminal](https://github.com/roloran/ROLORAN-Terminal) as RDCP-specific serial monitor solution, for example:

```bash
export LORADEV=/dev/cu.usbserial-0001
platformio run --target upload --upload-port $LORADEV
roloran-terminal.py
```

If you do not use our custom PCB, you will need to adjust the pinout in the source code accordingly to control the two SX1262 radios.

## Initial Configuration of the Two-Channel Relay

Before the two-channel relay will work as implemented, it needs to be configured via a series of serial commands, similar to the ROLODECK provisioning workflow.

However, while the RDCP two-channel relay configuration is similar to other ROLORAN LoRa device configuration procedures, additional care must be taken to appropriately address its two-channel nature.

The commands `SHOW CONFIG` and `RESET CONFIG` can be used similar to other ROLORAN devices. For initial configuration, the following commands *must* be used once:

- `LORAFREQ 433.175 868.200` sets the LoRa channel frequencies for both radios in MHz. Note that the `%.3f` format must be used.
- `LORABW 125 125` sets the LoRa channel bandwidth for both radios in kHz.
- `LORASF 07 12` sets the LoRa spreading factor for both radios. The use of leading zeros is mandatory for `SF < 10`.
- `LORACR 8 5` sets the LoRa coding rates for both radios in RadioLib format.
- `LORASW 12 34` sets the LoRa syncword for both radios; uses hexadecimal numbers.
- `LORAPW 05 07` sets the TX power for both radios in dBm; make sure to not violate any regional regulations in combination with the used cabling and antennas.
- `LORAPL 15 15` sets the LoRa preamble length for both radios in number of symbols.
- `RDCPADDR 0200` sets the RDCP address of the device. Make sure to use the appropriate RDCP address range for this kind of devices.
- `RDCPRLID 0` sets the RDCP Relay Identifier of the device. Double-check that this setting is correct and unique in the overall RDCP infrastructure or the whole mesh will drown in packet collisions.
- `RDCPNUMRL 10` sets the number of relays used in the current RDCP scenario.
- `RDCPRLOA 123` sets the three other relays to use in the OA direction. Make sure you have chosen a good plan for the whole RDCP infrastructure.
- `RDCPRLCR 34E` sets the three other relays to use in the CIRE direction. See note about `RDCPRLOA`.
- `RDCPTS7R DE` sets the Relay1 RDCP Header field value for Timeslot-7 transmission.
- `RDCPNEFF 0204` sets the neighbor to fetch messages from at start-up.
- `MULTICAST B000 B001 B002 0000 0000` set the five multicast RDCP addresses for this device.
- `NAME` gives the relay a name. This may be helpful for human operators.
- `HQPUBKEY` and `SHAREDSECRET` must be used as for ROLODECK devices.

After initial configuration, use the `RESTART` command along with `SHOW CONFIG` to verify that the configuration has been persisted properly.

Additional commands:

- `SHOW NEIGHBORS` lists the currently registered neighbors.
- `SHOW MEMORIES` lists the currently stored memories (old OAs and their Signatures).
- `TOGGLE something` enables or disables specific functionality. `something` can be `RELAY`, `EP`, `FORWARD`, `STATUS`, `FETCH`, `PERIODIC`, or `SEND`. Used by RDCP Infrastructure maintenance personnel only.
- `HEARTBEAT 30` sets the DA Heartbeat interval in minutes.
- `OMRETRANS 0` sets the initial value of the counter RDCP Header field when transmitting memories.
- `MAXPERAGE 24` sets the maximum age of memories selected for the Periodic868 chain.
- `PERIODICS 30` sets the Periodic868 chain kickstarter interval.
- `UNSOLICIT 180` sets the unsolicited DA Status Response interval.
- `RDCPDUPETABLERESET` resets the device's RDCP duplicate table, e.g., after changes to the overall infrastructure.
- `RDCPDUPETABLEDELETE` deletes the device's persisted duplicate table.
- `RDCPDUPETABLEZAP 020C` deletes the duplicate table entry for a specific RDCP address.
- `RDCPDUPETABLESET 020C 0123` sets the duplicate table entry for a specified RDCP address to the given value.
- `MAINTENANCE` switches the device into Maintenance Mode.
- `SERIAL text` sends the given text via UART to the hard-wire-connected `DA`.
- `CORRIDOR 60` sets the corridor basetime in seconds, i.e., how long to keep the channel free when hearing `CIRE` messages.
- `RDCPSFMUL 1` sets the spreading-factor-based time multiplier, e.g., 1 for SF7 and 10 for SF12.
- `RDCPSEQNR 1234` sets this device's most recently used RDCP Sequence Number, e.g., after replacing the ESP32 hardware.

## Interface to interactive DAs

The relay can optionally be connected to a DA via a serial / UART interface. Serial messages intended explicitly for the DA all start with a `DA_` prefix. DAs may send serial commands to the relay.

The following notifications are sent to the DA:

- `DA_TIME dd.mm.yyyy HH:MM (s)` when a new RDCP Timestamp is received. `s` is the RDCP Infrastructure status contained in RDCP Timestamp messages as decimal number.
- `DA_RDCP <base64>` when a RDCP Message is received that may be relevant for the DA, e.g., an OA, Signature, or a message addressed to this device.
- `DA_DELIVERY <address>` when a Delivery Receipt was received from the given neighbor.
- `DA_OA_RESET` when all OAs shall be reset.
- `DA_RESETDEVICE` when a device reset is requested.
- `DA_REBOOT` when a device reboot is requested.
- `DA_MAINTENANCE` when the DA is expected to switch into maintenance mode.
- `DA_INFRASTRUCTURE_RESET` when the whole RDCP infrastructure is reset.
- `DA_CIRESENT` when the TX callback for a CIRE is triggered.
- `DA_FETCHTIMEOUT` when no delivery receipt was received for an own memory fetch request.

DAs are expected to use the following serial commands for interaction with the relay:

- `BATTERY 100 085` is used to update the current status report for both DA batteries. Both values should be in the `000 -- 100` range and represent percentages based on sufficiently precise knowledge about the used BAT ADC and batteries. Those values are passed on to the HQ as a part of DA Status Responses. It is recommended to use this command roughly every 10 minutes. Note that leading zeros must be used for values `< 100` and `< 10`.
- `RDCPCIRE <subtype> <referencenumber> <textual content>` is used to send a new CIRE. `subtype` must be a 2-digit hex number, `referencenumber` a 4-digit hex number; the `textual content` format should match MGs' and be plain ASCII text without line breaks. Note that the Relay takes care of Unishox2 compression and authenticated encryption. DAs are expected to send CIREs again in case of timeouts (no `DA_CIRESENT` or HQ ACK received).
- `RDCPFETCH 12EF` fetches a single message identified by its reference number from the designated neighbor. Result may consist of multiple fragments and typically includes signature.

DAs may use assigned virtual RDCP addresses along with the `SIMRX 433` and `SIMRX 868` commands to trigger the forwarding, relaying, or processing of arbitrary other RDCP messages if deemed an actual necessity. DAs must not use the relay as LoRa modem and especially not interfere otherwise with the sequence numbers used for the relay's RDCP address.

DAs also must implement their own duplicate handling as they may receive the
same OAs, Signatures, or other RDCP Messages multiple times.

Finally, DAs are expected to implement their own RDCP Message Board functionality and honor the handling of multiple OA fragments, lifetimes, and lifetime updates independently of the Relay.

## Specific implementation aspects for RDCP Infrastructure HQs

The following implementation specifics need to be kept in mind when processing RDCP messages originating at the Relay:

- Reported `RSSI` values have a bias/offset of `200`. Thus, if the real RSSI value is `-130`, it will be reported as `70`.
- Reported `SNR` values have a bias/offset of `100`. Thus, if the real SNR value is `6`, it will be reported as `106`.
- The 16-bit battery status value reported in DA Status Response messages actually consists of the two unsigned 8-bit values set by the `BATTERY` serial command or has the magic value of `0xFFFF` if those were not set since the last counter reset.

<!-- EOF -->
