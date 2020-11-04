
"""
`adafruit_ssd1306`
====================================================
MicroPython SSD1306 OLED driver, I2C and SPI interfaces
* Author(s): Tony DiCola, Michael McWethy
"""

import time

from micropython import const
#from adafruit_bus_device import i2c_device, spi_device

try:
    import framebuf
except ImportError:
    import adafruit_framebuf as framebuf

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SSD1306.git"

# register definitions
SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_DISP = const(0xAE)
SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)

class I2CDevice:
    """
    Represents a single I2C device and manages locking the bus and the device
    address.
    :param ~busio.I2C i2c: The I2C bus the device is on
    :param int device_address: The 7 bit device address
    :param bool probe: Probe for the device upon object creation, default is true
    .. note:: This class is **NOT** built into CircuitPython. See
      :ref:`here for install instructions <bus_device_installation>`.
    Example:
    .. code-block:: python
        import busio
        from board import *
        from adafruit_bus_device.i2c_device import I2CDevice
        with busio.I2C(SCL, SDA) as i2c:
            device = I2CDevice(i2c, 0x70)
            bytes_read = bytearray(4)
            with device:
                device.readinto(bytes_read)
            # A second transaction
            with device:
                device.write(bytes_read)
    """

    def __init__(self, i2c, device_address, probe=True):

        self.i2c = i2c
        self.device_address = device_address

        if probe:
            self.__probe_for_device()

    def readinto(self, buf, *, start=0, end=None):
        """
        Read into ``buf`` from the device. The number of bytes read will be the
        length of ``buf``.
        If ``start`` or ``end`` is provided, then the buffer will be sliced
        as if ``buf[start:end]``. This will not cause an allocation like
        ``buf[start:end]`` will so it saves memory.
        :param bytearray buffer: buffer to write into
        :param int start: Index to start writing at
        :param int end: Index to write up to but not include; if None, use ``len(buf)``
        """
        if end is None:
            end = len(buf)
        self.i2c.readfrom_into(self.device_address, buf, start=start, end=end)

    def write(self, buf, *, start=0, end=None):
        """
        Write the bytes from ``buffer`` to the device, then transmit a stop
        bit.
        If ``start`` or ``end`` is provided, then the buffer will be sliced
        as if ``buffer[start:end]``. This will not cause an allocation like
        ``buffer[start:end]`` will so it saves memory.
        :param bytearray buffer: buffer containing the bytes to write
        :param int start: Index to start writing from
        :param int end: Index to read up to but not include; if None, use ``len(buf)``
        """
        if end is None:
            end = len(buf)
        self.i2c.writeto(self.device_address, buf, start=start, end=end)

    # pylint: disable-msg=too-many-arguments
    def write_then_readinto(
        self,
        out_buffer,
        in_buffer,
        *,
        out_start=0,
        out_end=None,
        in_start=0,
        in_end=None
    ):
        """
        Write the bytes from ``out_buffer`` to the device, then immediately
        reads into ``in_buffer`` from the device. The number of bytes read
        will be the length of ``in_buffer``.
        If ``out_start`` or ``out_end`` is provided, then the output buffer
        will be sliced as if ``out_buffer[out_start:out_end]``. This will
        not cause an allocation like ``buffer[out_start:out_end]`` will so
        it saves memory.
        If ``in_start`` or ``in_end`` is provided, then the input buffer
        will be sliced as if ``in_buffer[in_start:in_end]``. This will not
        cause an allocation like ``in_buffer[in_start:in_end]`` will so
        it saves memory.
        :param bytearray out_buffer: buffer containing the bytes to write
        :param bytearray in_buffer: buffer containing the bytes to read into
        :param int out_start: Index to start writing from
        :param int out_end: Index to read up to but not include; if None, use ``len(out_buffer)``
        :param int in_start: Index to start writing at
        :param int in_end: Index to write up to but not include; if None, use ``len(in_buffer)``
        """
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)

        self.i2c.writeto_then_readfrom(
            self.device_address,
            out_buffer,
            in_buffer,
            out_start=out_start,
            out_end=out_end,
            in_start=in_start,
            in_end=in_end,
        )

    # pylint: enable-msg=too-many-arguments

    def __enter__(self):
        while not self.i2c.try_lock():
            pass
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.i2c.unlock()
        return False

    def __probe_for_device(self):
        """
        Try to read a byte from an address,
        if you get an OSError it means the device is not there
        or that the device does not support these means of probing
        """
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.device_address, b"")
        except OSError:
            # some OS's dont like writing an empty bytesting...
            # Retry by reading a byte
            try:
                result = bytearray(1)
                self.i2c.readfrom_into(self.device_address, result)
            except OSError:
                raise ValueError("No I2C device at address: %x" % self.device_address)
        finally:
            self.i2c.unlock()


class _SSD1306(framebuf.FrameBuffer):
    """Base class for SSD1306 display driver"""

    # pylint: disable-msg=too-many-arguments
    def __init__(self, buffer, width, height, *, external_vcc, reset):
        super().__init__(buffer, width, height)
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        # reset may be None if not needed
        self.reset_pin = reset
        if self.reset_pin:
            self.reset_pin.switch_to_output(value=0)
        self.pages = self.height // 8
        # Note the subclass must initialize self.framebuf to a framebuffer.
        # This is necessary because the underlying data buffer is different
        # between I2C and SPI implementations (I2C needs an extra byte).
        self._power = False
        self.poweron()
        self.init_display()

    @property
    def power(self):
        """True if the display is currently powered on, otherwise False"""
        return self._power

    def init_display(self):
        """Base class to initialize display"""
        for cmd in (
            SET_DISP | 0x00,  # off
            # address setting
            SET_MEM_ADDR,
            0x00,  # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,  # column addr 127 mapped to SEG0
            SET_MUX_RATIO,
            self.height - 1,
            SET_COM_OUT_DIR | 0x08,  # scan from COM[N] to COM0
            SET_DISP_OFFSET,
            0x00,
            SET_COM_PIN_CFG,
            0x02 if self.height == 32 or self.height == 16 else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV,
            0x80,
            SET_PRECHARGE,
            0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL,
            0x30,  # 0.83*Vcc
            # display
            SET_CONTRAST,
            0xFF,  # maximum
            SET_ENTIRE_ON,  # output follows RAM contents
            SET_NORM_INV,  # not inverted
            # charge pump
            SET_CHARGE_PUMP,
            0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01,
        ):  # on
            self.write_cmd(cmd)
        if self.width == 72:
            self.write_cmd(0xAD)
            self.write_cmd(0x30)
        self.fill(0)
        self.show()

    def poweroff(self):
        """Turn off the display (nothing visible)"""
        self.write_cmd(SET_DISP | 0x00)
        self._power = False

    def contrast(self, contrast):
        """Adjust the contrast"""
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        """Invert all pixels on the display"""
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def write_framebuf(self):
        """Derived class must implement this"""
        raise NotImplementedError

    def write_cmd(self, cmd):
        """Derived class must implement this"""
        raise NotImplementedError

    def poweron(self):
        "Reset device and turn on the display."
        if self.reset_pin:
            self.reset_pin.value = 1
            time.sleep(0.001)
            self.reset_pin.value = 0
            time.sleep(0.010)
            self.reset_pin.value = 1
            time.sleep(0.010)
        self.write_cmd(SET_DISP | 0x01)
        self._power = True

    def show(self):
        """Update the display"""
        xpos0 = 0
        xpos1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            xpos0 += 32
            xpos1 += 32
        if self.width == 72:
            # displays with width of 72 pixels are shifted by 28
            xpos0 += 28
            xpos1 += 28
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(xpos0)
        self.write_cmd(xpos1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()


class SSD1306_I2C(_SSD1306):
    """
    I2C class for SSD1306
    :param width: the width of the physical screen in pixels,
    :param height: the height of the physical screen in pixels,
    :param i2c: the I2C peripheral to use,
    :param addr: the 8-bit bus address of the device,
    :param external_vcc: whether external high-voltage source is connected.
    :param reset: if needed, DigitalInOut designating reset pin
    """

    def __init__(
        self, width, height, i2c, *, addr=0x3C, external_vcc=False, reset=None
    ):
        self.i2c_device = i2c_device.I2CDevice(i2c, addr)
        self.addr = addr
        self.temp = bytearray(2)
        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        super().__init__(
            memoryview(self.buffer)[1:],
            width,
            height,
            external_vcc=external_vcc,
            reset=reset,
        )

    def write_cmd(self, cmd):
        """Send a command to the SPI device"""
        self.temp[0] = 0x80  # Co=1, D/C#=0
        self.temp[1] = cmd
        with self.i2c_device:
            self.i2c_device.write(self.temp)

    def write_framebuf(self):
        """Blast out the frame buffer using a single I2C transaction to support
        hardware I2C interfaces."""
        with self.i2c_device:
            self.i2c_device.write(self.buffer)


# pylint: disable-msg=too-many-arguments
class SSD1306_SPI(_SSD1306):
    """
    SPI class for SSD1306
    :param width: the width of the physical screen in pixels,
    :param height: the height of the physical screen in pixels,
    :param spi: the SPI peripheral to use,
    :param dc: the data/command pin to use (often labeled "D/C"),
    :param reset: the reset pin to use,
    :param cs: the chip-select pin to use (sometimes labeled "SS").
    """

    # pylint: disable=no-member
    # Disable should be reconsidered when refactor can be tested.
    def __init__(
        self,
        width,
        height,
        spi,
        dc,
        reset,
        cs,
        *,
        external_vcc=False,
        baudrate=8000000,
        polarity=0,
        phase=0
    ):
        self.rate = 10 * 1024 * 1024
        dc.switch_to_output(value=0)
        self.spi_device = spi_device.SPIDevice(
            spi, cs, baudrate=baudrate, polarity=polarity, phase=phase
        )
        self.dc_pin = dc
        self.buffer = bytearray((height // 8) * width)
        super().__init__(
            memoryview(self.buffer),
            width,
            height,
            external_vcc=external_vcc,
            reset=reset,
        )

    def write_cmd(self, cmd):
        """Send a command to the SPI device"""
        self.dc_pin.value = 0
        with self.spi_device as spi:
            spi.write(bytearray([cmd]))

    def write_framebuf(self):
        """write to the frame buffer via SPI"""
        self.dc_pin.value = 1
        with self.spi_device as spi:
            spi.write(self.buffer)



# Import all board pins.
from machine import Pin, I2C
#import busio

# Import the SSD1306 module.
#import adafruit_ssd1306


# Create the I2C interface.
#i2c = busio.I2C(SCL, SDA)
i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)

display = SSD1306_I2C(128, 32, i2c, addr=0x31)

display.fill(0)
display.show()

# Set a pixel in the origin 0,0 position.
display.pixel(0, 0, 1)
# Set a pixel in the middle 64, 16 position.
display.pixel(64, 16, 1)
# Set a pixel in the opposite 127, 31 position.
display.pixel(127, 31, 1)
display.show()
