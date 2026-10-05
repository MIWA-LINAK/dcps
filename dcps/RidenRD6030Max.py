#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Control a Riden RD6030-Max power supply over its USB serial interface.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    from .SCPI import SCPI
except (ImportError, ValueError):
    from SCPI import SCPI

import time
import math
import pyvisa as visa
from struct import pack, unpack
from time import sleep


class RidenRD6030Max(SCPI):
    """Control a Riden RD6030-Max using Modbus RTU over USB serial."""

    VOLTAGE_REGISTER_SCALE = 100
    CURRENT_REGISTER_SCALE = 100
    MAX_VOLTAGE = 60.0
    MAX_CURRENT = 30.0

    def __init__(self, resource, wait=1.0, verbosity=0, address=1,
                 voltage_scale=VOLTAGE_REGISTER_SCALE,
                 current_scale=CURRENT_REGISTER_SCALE, **kwargs):
        """Initialize the power supply connection.

        resource       - VISA serial resource, for example ASRL3::INSTR
        wait           - seconds to wait after setting a value or output state
        verbosity      - verbosity output; set to 0 for no debug output
        address        - Modbus RTU device address (default 1)
        voltage_scale  - register counts per volt (default 100)
        current_scale  - register counts per amp (default 100)
        kwargs         - other named options passed to PyVISA open_resource()
        """
        if isinstance(address, bool) or not isinstance(address, int):
            raise ValueError("Modbus address must be an integer")
        if not 1 <= address <= 247:
            raise ValueError("Modbus address must be between 1 and 247")
        if (not math.isfinite(voltage_scale) or
                not math.isfinite(current_scale) or
                voltage_scale <= 0 or current_scale <= 0):
            raise ValueError("Register scales must be greater than zero")

        kwargs.setdefault("baud_rate", 115200)
        kwargs.setdefault("data_bits", 8)
        kwargs.setdefault("parity", visa.constants.Parity.none)
        kwargs.setdefault("stop_bits", visa.constants.StopBits.one)

        self._address = address
        self._voltage_scale = voltage_scale
        self._current_scale = current_scale
        super(RidenRD6030Max, self).__init__(
            resource,
            max_chan=1,
            wait=wait,
            verbosity=verbosity,
            read_termination="",
            write_termination="",
            **kwargs
        )

    @staticmethod
    def _crc16(data):
        """Return the Modbus CRC-16 bytes in transmission order."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return pack("<H", crc)

    def _read_exact(self, count):
        response = self._inst.read_bytes(count=count, chunk_size=1)
        if len(response) != count:
            raise IOError("Incomplete Modbus RTU response")
        return response

    def _read_response(self, function):
        header = self._read_exact(2)
        if header[1] & 0x80:
            return header + self._read_exact(3)
        if header[1] != function:
            raise IOError("Unexpected Modbus function in response")
        if function == 3:
            byte_count = self._read_exact(1)
            return header + byte_count + self._read_exact(byte_count[0] + 2)
        return header + self._read_exact(6)

    def _exchange(self, function, payload):
        packet = pack(">BB", self._address, function) + payload
        self._inst.write_raw(packet + self._crc16(packet))
        response = self._read_response(function)

        if self._crc16(response[:-2]) != response[-2:]:
            raise IOError("Invalid Modbus RTU response CRC")
        if response[0] != self._address:
            raise IOError("Modbus response address does not match")
        if response[1] == (function | 0x80):
            raise RuntimeError(
                "Modbus exception response: {}".format(response[2])
            )
        if response[1] != function:
            raise IOError("Unexpected Modbus function in response")
        return response[:-2]

    def _read_registers(self, start, count):
        if not 0 <= start <= 0xFFFF or not 1 <= count <= 125:
            raise ValueError("Invalid Modbus register range")
        response = self._exchange(3, pack(">HH", start, count))
        if response[2] != count * 2:
            raise IOError("Unexpected Modbus register response length")
        return list(unpack(">" + "H" * count, response[3:]))

    def _read_register(self, register):
        return self._read_registers(register, 1)[0]

    def _write_register(self, register, value):
        if not 0 <= register <= 0xFFFF or not 0 <= value <= 0xFFFF:
            raise ValueError("Invalid Modbus register or value")
        expected = pack(">BBHH", self._address, 6, register, value)
        response = self._exchange(6, pack(">HH", register, value))
        if response != expected:
            raise IOError("Modbus write response does not match request")

    def _select_channel(self, channel):
        if channel is not None:
            self.channel = channel

    @staticmethod
    def _wait_after(wait, default_wait):
        if wait is None:
            wait = default_wait
        if wait > 0:
            sleep(wait)

    def _encode_setting(self, value, maximum, scale, name):
        value = float(value)
        if not math.isfinite(value) or value < 0 or value > maximum:
            raise ValueError("{} must be between 0 and {}".format(name, maximum))
        return int(round(value * scale))

    def idn(self):
        """Return the model, serial number, and firmware from device registers."""
        registers = self._read_registers(0, 4)
        serial_number = (registers[1] << 16) | registers[2]
        firmware = registers[3] / 100.0
        return "RIDEN,RD6030-Max,{},{}".format(serial_number, firmware)

    def setVoltage(self, voltage, channel=None, wait=None):
        """Set the voltage limit in volts."""
        self._select_channel(channel)
        value = self._encode_setting(
            voltage, self.MAX_VOLTAGE, self._voltage_scale, "Voltage"
        )
        self._write_register(8, value)
        self._wait_after(wait, self._wait)

    def setCurrent(self, current, channel=None, wait=None):
        """Set the current limit in amperes."""
        self._select_channel(channel)
        value = self._encode_setting(
            current, self.MAX_CURRENT, self._current_scale, "Current"
        )
        self._write_register(9, value)
        self._wait_after(wait, self._wait)

    def queryVoltage(self, channel=None):
        """Return the configured voltage limit in volts."""
        self._select_channel(channel)
        return self._read_register(8) / self._voltage_scale

    def queryCurrent(self, channel=None):
        """Return the configured current limit in amperes."""
        self._select_channel(channel)
        return self._read_register(9) / self._current_scale

    def measureVoltage(self, channel=None):
        """Return the measured output voltage in volts."""
        self._select_channel(channel)
        return self._read_register(10) / self._voltage_scale

    def measureCurrent(self, channel=None):
        """Return the measured output current in amperes."""
        self._select_channel(channel)
        return self._read_register(11) / self._current_scale

    def isOutputOn(self, channel=None):
        """Return whether the output is enabled."""
        self._select_channel(channel)
        return bool(self._read_register(18))

    def outputOn(self, channel=None, wait=None):
        """Enable the output."""
        self._select_channel(channel)
        self._write_register(18, 1)
        self._wait_after(wait, self._wait)

    def outputOff(self, channel=None, wait=None):
        """Disable the output."""
        self._select_channel(channel)
        self._write_register(18, 0)
        self._wait_after(wait, self._wait)

    def outputOnAll(self, wait=None):
        """Enable the single output."""
        self.outputOn(wait=wait)

    def outputOffAll(self, wait=None):
        """Disable the single output."""
        self.outputOff(wait=wait)


if __name__ == "__main__":
    import os

    resource = os.environ.get("RD6030_MAX_VISA", "ASRL11::INSTR")
    psu = RidenRD6030Max(resource)
    psu.open()
    try:
        old_v = psu.queryVoltage()
        old_i = psu.queryCurrent()
        print("Device: {}".format(psu.idn()))
        print("Set voltage: {:.2f} V".format(psu.queryVoltage()))
        print("Set current: {:.2f} A".format(psu.queryCurrent()))
        psu.setVoltage(10)
        psu.setCurrent(0.5)
        print("Set voltage: {:.2f} V".format(psu.queryVoltage()))
        print("Set current: {:.2f} A".format(psu.queryCurrent()))
        print("Measured voltage: {:.2f} V".format(psu.measureVoltage()))
        print("Measured current: {:.2f} A".format(psu.measureCurrent()))
        print("Output enabled: {}".format(psu.isOutputOn()))
        time.sleep(1)
        psu.setVoltage(old_v)
        psu.setCurrent(old_i)
    finally:
        psu.close()
