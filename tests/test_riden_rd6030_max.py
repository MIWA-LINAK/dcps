import unittest

from dcps.RidenRD6030Max import RidenRD6030Max


class FakeInstrument:
    def __init__(self):
        self.registers = {
            8: 1200,
            9: 150,
            10: 1198,
            11: 148,
            18: 1,
        }
        self.response = b""
        self.requests = []
        self.corrupt_crc = False
        self.exception_code = None

    def write_raw(self, packet):
        self.requests.append(packet)
        address, function = packet[:2]
        if self.exception_code is not None:
            body = bytes((address, function | 0x80, self.exception_code))
        elif function == 3:
            start = int.from_bytes(packet[2:4], "big")
            count = int.from_bytes(packet[4:6], "big")
            payload = b"".join(
                self.registers.get(register, 0).to_bytes(2, "big")
                for register in range(start, start + count)
            )
            body = bytes((address, function, len(payload))) + payload
        else:
            register = int.from_bytes(packet[2:4], "big")
            value = int.from_bytes(packet[4:6], "big")
            self.registers[register] = value
            body = packet[:-2]
        self.response = body + RidenRD6030Max._crc16(body)
        if self.corrupt_crc:
            self.response = self.response[:-1] + bytes((self.response[-1] ^ 0xFF,))

    def read_bytes(self, count, chunk_size=1):
        response, self.response = self.response[:count], self.response[count:]
        return response


class RidenRD6030MaxTests(unittest.TestCase):
    def setUp(self):
        self.psu = RidenRD6030Max("ASRL1::INSTR", wait=0)
        self.psu._inst = FakeInstrument()

    def test_modbus_crc(self):
        self.assertEqual(
            RidenRD6030Max._crc16(bytes.fromhex("01 03 00 00 00 01")),
            bytes.fromhex("84 0A"),
        )

    def test_setting_and_querying_voltage_and_current(self):
        self.psu.setVoltage(12.34)
        self.psu.setCurrent(1.25)

        self.assertEqual(self.psu._inst.registers[8], 1234)
        self.assertEqual(self.psu._inst.registers[9], 125)
        self.assertEqual(self.psu.queryVoltage(), 12.34)
        self.assertEqual(self.psu.queryCurrent(), 1.25)

    def test_measurements_and_output_state(self):
        self.assertEqual(self.psu.measureVoltage(), 11.98)
        self.assertEqual(self.psu.measureCurrent(), 1.48)
        self.assertTrue(self.psu.isOutputOn())

        self.psu.outputOff()
        self.assertFalse(self.psu.isOutputOn())

    def test_rejects_out_of_range_settings_without_writing(self):
        with self.assertRaises(ValueError):
            self.psu.setVoltage(60.01)
        with self.assertRaises(ValueError):
            self.psu.setCurrent(-0.1)
        self.assertEqual(self.psu._inst.requests, [])

    def test_rejects_invalid_crc_and_modbus_exception(self):
        self.psu._inst.corrupt_crc = True
        with self.assertRaisesRegex(IOError, "CRC"):
            self.psu.queryVoltage()

        self.psu._inst.corrupt_crc = False
        self.psu._inst.exception_code = 2
        with self.assertRaisesRegex(RuntimeError, "exception response: 2"):
            self.psu.queryVoltage()


if __name__ == "__main__":
    unittest.main()
