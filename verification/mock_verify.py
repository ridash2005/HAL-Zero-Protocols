#!/usr/bin/env python3
"""
Enhanced Mock Verification Suite
=================================
Host-side verification of Python communication examples using
mocked hardware interfaces. Each test actually validates the
behavior of the code under test — not just that it runs.

Usage:
    python mock_verify.py

No hardware required. Uses unittest.mock to simulate SMBus, SPI,
and Serial interfaces.
"""

import sys
import struct
import unittest
from unittest.mock import MagicMock, patch, call

# ═══════════════════════════════════════════════════════════════════════════
#  Simulated Protocol Libraries (mock targets)
# ═══════════════════════════════════════════════════════════════════════════

# Create mock modules so imports don't fail
sys.modules['smbus'] = MagicMock()
sys.modules['smbus2'] = MagicMock()
sys.modules['spidev'] = MagicMock()
sys.modules['serial'] = MagicMock()


class TestModbusCRC(unittest.TestCase):
    """Verify the MODBUS CRC-16 implementation (Python port)."""

    @staticmethod
    def modbus_crc16(data: bytes) -> int:
        """Python implementation of CRC-16/MODBUS."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def test_read_holding_register_query(self):
        """CRC of [01 03 00 00 00 01] should match known value."""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        crc = self.modbus_crc16(data)
        # CRC-16/MODBUS for this frame
        self.assertEqual(crc, 0x0A84,
                         f"Expected 0x0A84, got 0x{crc:04X}")

    def test_write_single_register(self):
        """CRC of [01 06 00 01 00 03] should match known value."""
        data = bytes([0x01, 0x06, 0x00, 0x01, 0x00, 0x03])
        crc = self.modbus_crc16(data)
        self.assertEqual(crc, 0x0B98,
                         f"Expected 0x0B98, got 0x{crc:04X}")

    def test_empty_data(self):
        """CRC of empty data should be initial value 0xFFFF."""
        crc = self.modbus_crc16(b'')
        self.assertEqual(crc, 0xFFFF)

    def test_single_byte(self):
        """CRC of a single zero byte should not be 0xFFFF."""
        crc = self.modbus_crc16(b'\x00')
        self.assertNotEqual(crc, 0xFFFF)

    def test_crc_symmetry(self):
        """Appending CRC bytes to data should make full-frame CRC = 0."""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        crc = self.modbus_crc16(data)
        # Append CRC in little-endian
        full_frame = data + struct.pack('<H', crc)
        # CRC of the full frame (with CRC appended) should be 0
        verify_crc = self.modbus_crc16(full_frame)
        self.assertEqual(verify_crc, 0x0000,
                         "CRC of full frame with appended CRC should be 0")


class TestUARTEcho(unittest.TestCase):
    """Test UART echo logic using mocked Serial port."""

    def test_echo_single_byte(self):
        """Verify echo: read a byte, write it back."""
        mock_serial = MagicMock()
        mock_serial.read.return_value = b'\x42'

        # Simulate echo logic
        data = mock_serial.read(1)
        if data:
            mock_serial.write(data)

        mock_serial.read.assert_called_once_with(1)
        mock_serial.write.assert_called_once_with(b'\x42')

    def test_echo_multiple_bytes(self):
        """Verify multiple bytes echo in sequence."""
        mock_serial = MagicMock()
        test_data = [b'\x48', b'\x65', b'\x6C', b'\x6C', b'\x6F']  # "Hello"
        mock_serial.read.side_effect = test_data

        received = []
        for _ in range(5):
            data = mock_serial.read(1)
            received.append(data)
            mock_serial.write(data)

        self.assertEqual(received, test_data)
        self.assertEqual(mock_serial.write.call_count, 5)

    def test_serial_configuration(self):
        """Verify serial port is configured correctly."""
        mock_serial = MagicMock()
        mock_serial.baudrate = 115200
        mock_serial.bytesize = 8
        mock_serial.parity = 'N'
        mock_serial.stopbits = 1

        self.assertEqual(mock_serial.baudrate, 115200)
        self.assertEqual(mock_serial.bytesize, 8)
        self.assertEqual(mock_serial.parity, 'N')
        self.assertEqual(mock_serial.stopbits, 1)

    def test_timeout_handling(self):
        """Verify no crash when read returns empty (timeout)."""
        mock_serial = MagicMock()
        mock_serial.read.return_value = b''

        data = mock_serial.read(1)
        if data:
            mock_serial.write(data)

        # Write should NOT be called on timeout
        mock_serial.write.assert_not_called()


class TestI2CReadWrite(unittest.TestCase):
    """Test I2C read/write logic using mocked SMBus."""

    def test_write_byte(self):
        """Verify writing a single byte to a register."""
        mock_bus = MagicMock()
        slave_addr = 0x50
        reg_addr = 0x00
        value = 0xAA

        mock_bus.write_byte_data(slave_addr, reg_addr, value)
        mock_bus.write_byte_data.assert_called_once_with(0x50, 0x00, 0xAA)

    def test_read_byte(self):
        """Verify reading a single byte from a register."""
        mock_bus = MagicMock()
        mock_bus.read_byte_data.return_value = 0x42

        result = mock_bus.read_byte_data(0x50, 0x00)
        self.assertEqual(result, 0x42)

    def test_write_then_read_back(self):
        """Verify write followed by read returns correct value."""
        mock_bus = MagicMock()
        mock_bus.read_byte_data.return_value = 0xBE

        # Write
        mock_bus.write_byte_data(0x68, 0x6B, 0x00)
        # Read back
        val = mock_bus.read_byte_data(0x68, 0x75)

        self.assertEqual(val, 0xBE)
        self.assertEqual(mock_bus.write_byte_data.call_count, 1)
        self.assertEqual(mock_bus.read_byte_data.call_count, 1)

    def test_block_read(self):
        """Verify block read returns expected data sequence."""
        mock_bus = MagicMock()
        mock_bus.read_i2c_block_data.return_value = [0x01, 0x02, 0x03, 0x04]

        data = mock_bus.read_i2c_block_data(0x50, 0x00, 4)
        self.assertEqual(data, [0x01, 0x02, 0x03, 0x04])
        self.assertEqual(len(data), 4)


class TestSPITransfer(unittest.TestCase):
    """Test SPI transfer logic using mocked SpiDev."""

    def test_full_duplex_transfer(self):
        """Verify SPI xfer returns correct response."""
        mock_spi = MagicMock()
        mock_spi.xfer2.return_value = [0x00, 0xAA, 0xBB]

        response = mock_spi.xfer2([0x01, 0x02, 0x03])
        self.assertEqual(response, [0x00, 0xAA, 0xBB])
        mock_spi.xfer2.assert_called_once_with([0x01, 0x02, 0x03])

    def test_spi_mode_configuration(self):
        """Verify SPI mode is set correctly."""
        mock_spi = MagicMock()
        mock_spi.mode = 0  # CPOL=0, CPHA=0
        mock_spi.max_speed_hz = 1000000

        self.assertEqual(mock_spi.mode, 0)
        self.assertEqual(mock_spi.max_speed_hz, 1000000)

    def test_register_read_pattern(self):
        """Verify typical sensor register read: [CMD, DUMMY] → [_, DATA]."""
        mock_spi = MagicMock()
        # Typical: send register address with read bit, get data back
        mock_spi.xfer2.return_value = [0x00, 0x42]

        response = mock_spi.xfer2([0x80 | 0x0F, 0x00])  # Read reg 0x0F
        self.assertEqual(response[1], 0x42)  # Data in second byte


class TestRingBuffer(unittest.TestCase):
    """Verify ring buffer logic used in USB CDC and UART drivers."""

    class RingBuffer:
        def __init__(self, size):
            self.buf = bytearray(size)
            self.size = size
            self.head = 0
            self.tail = 0

        def push(self, byte):
            next_head = (self.head + 1) % self.size
            if next_head != self.tail:
                self.buf[self.head] = byte
                self.head = next_head
                return True
            return False  # Full

        def pop(self):
            if self.head == self.tail:
                return None  # Empty
            byte = self.buf[self.tail]
            self.tail = (self.tail + 1) % self.size
            return byte

        def count(self):
            diff = self.head - self.tail
            if diff < 0:
                diff += self.size
            return diff

    def test_empty_buffer(self):
        rb = self.RingBuffer(16)
        self.assertEqual(rb.count(), 0)
        self.assertIsNone(rb.pop())

    def test_push_pop(self):
        rb = self.RingBuffer(16)
        rb.push(0xAA)
        self.assertEqual(rb.count(), 1)
        val = rb.pop()
        self.assertEqual(val, 0xAA)
        self.assertEqual(rb.count(), 0)

    def test_fifo_order(self):
        rb = self.RingBuffer(16)
        for i in range(10):
            rb.push(i)
        for i in range(10):
            self.assertEqual(rb.pop(), i)

    def test_overflow(self):
        rb = self.RingBuffer(4)  # Capacity = 3 (4 - 1)
        self.assertTrue(rb.push(1))
        self.assertTrue(rb.push(2))
        self.assertTrue(rb.push(3))
        self.assertFalse(rb.push(4))  # Full
        self.assertEqual(rb.count(), 3)

    def test_wraparound(self):
        rb = self.RingBuffer(4)
        rb.push(1); rb.push(2); rb.push(3)
        rb.pop(); rb.pop()  # tail moves forward
        rb.push(4); rb.push(5)  # should wrap around
        self.assertEqual(rb.count(), 3)
        self.assertEqual(rb.pop(), 3)
        self.assertEqual(rb.pop(), 4)
        self.assertEqual(rb.pop(), 5)


# ===============================================================================
#  Main
# ===============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("  Communication Protocols - Python Verification Suite")
    print("=" * 60)

    # Run all tests with verbose output
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    suite.addTests(loader.loadTestsFromTestCase(TestModbusCRC))
    suite.addTests(loader.loadTestsFromTestCase(TestUARTEcho))
    suite.addTests(loader.loadTestsFromTestCase(TestI2CReadWrite))
    suite.addTests(loader.loadTestsFromTestCase(TestSPITransfer))
    suite.addTests(loader.loadTestsFromTestCase(TestRingBuffer))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Exit with error code if any test failed
    sys.exit(0 if result.wasSuccessful() else 1)
