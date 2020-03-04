"""BME280 I2C driver for MicroPython.

Implemented in accordance with the datasheet from Bosch.
https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf

The functions for calculating representive values from the ADC data returned
by the chip are lifted from the datasheet.

At present this driver does not implement functionality to take advantage of
the onboard IIR filter.

Extending this driver to support SPI should be trivial, a BME280 board wired
for SPI was not available to the author at the time.

Two modes of operation exist for this sensor, forced mode and normal mode.
In forced mode the sensor is in standby until it is instructed to record a
sample. In normal mode the sensor will cycle between recoding a sample and
standby for a configurable period.

Example usage in forced read mode:

    from machine import I2C, Pin
    i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
    bme = BME280(i2c)
    bme.read()
    print(bme.temperature, '°C')
    print(bme.pressure/100, 'hPa')
    print(bme.humidity(), '%RH')

Example usage in normal mode:
    from machine import I2C, Pin
    i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
    bme = BME280(i2c)
    bme.read() # Use force to also set oversample, could be specified.
    bme.settings.config.t_sb = STANDBY.index(1000) # 1000ms standby
    bme.settings.ctrl_meas.mode = MODE_NORMAL
    bme.write_settings()
    sample_rate = bme.delay / 1000
    while True:
        bme.read(force=False)
        utime.sleep(sample_rate)

"""
import ctypes
from micropython import const
import utime

DEV_ADDR = const(0x76)
ADDR_CALA = const(0x88)
ADDR_CALB = const(0xE1)
ADDR_DATA = const(0xF7)
ADDR_SETTINGS = const(0xF2)
MODE_FORCED = const(1)
MODE_NORMAL = const(3)
# Index of OVERSAMPLE represents integer setting for oversample multiplier
# e.g. setting BME280 to oversample setting of `4` is 8x oversample.
OVERSAMPLE = (0, 1, 2, 4, 8, 16)
# Index of STANDBY represents integer setting for standby ms
# e.g. setting BME280 to standby of `3` is 250ms
STANDBY = (0.5, 62.5, 125, 250, 500, 1000, 10, 20)

SETTINGS = {
    'ctrl_hum': (0, {
        'osrs_h': ctypes.BFUINT8 | 0 << ctypes.BF_POS | 3 << ctypes.BF_LEN,
    }),
    'status': (1, {
        'im_update': ctypes.BFUINT8 | 0 << ctypes.BF_POS | 1 << ctypes.BF_LEN,
        'measuring': ctypes.BFUINT8 | 3 << ctypes.BF_POS | 1 << ctypes.BF_LEN,
    }),
    'ctrl_meas': (2, {
        'mode': ctypes.BFUINT8 | 0 << ctypes.BF_POS | 2 << ctypes.BF_LEN,
        'osrs_p': ctypes.BFUINT8 | 2 << ctypes.BF_POS | 3 << ctypes.BF_LEN,
        'osrs_t': ctypes.BFUINT8 | 5 << ctypes.BF_POS | 3 << ctypes.BF_LEN,
    }),
    'config': (3, {
        'spi3w_en': ctypes.BFUINT8 | 0 << ctypes.BF_POS | 1 << ctypes.BF_LEN,
        'filter': ctypes.BFUINT8 | 2 << ctypes.BF_POS | 3 << ctypes.BF_LEN,
        't_sb': ctypes.BFUINT8 | 5 << ctypes.BF_POS | 3 << ctypes.BF_LEN,
    })
}

DATA = {
    'adc_p': 0 | ctypes.BFUINT32 | 12 << ctypes.BF_POS | 20 << ctypes.BF_LEN,
    'adc_t': 3 | ctypes.BFUINT32 | 12 << ctypes.BF_POS | 20 << ctypes.BF_LEN,
    'adc_h': 6 | ctypes.UINT16,
}

CALIBRATION = {
    # Loaded from 0x88
    'T1': 0 | ctypes.UINT16,
    'T2': 2 | ctypes.INT16,
    'T3': 4 | ctypes.INT16,
    'P1': 6 | ctypes.UINT16,
    # Referencing array is messy, just list 'em
    # 'P2_9': (8 | ctypes.ARRAY, 8 | ctypes.INT16),
    'P2': 8 | ctypes.INT16,
    'P3': 10 | ctypes.INT16,
    'P4': 12 | ctypes.INT16,
    'P5': 14 | ctypes.INT16,
    'P6': 16 | ctypes.INT16,
    'P7': 18 | ctypes.INT16,
    'P8': 20 | ctypes.INT16,
    'P9': 22 | ctypes.INT16,
    'H1': 25 | ctypes.UINT8,
    # Following 7 bytes loaded starting at 0xE1
    'H2': 26 | ctypes.INT16,
    'H3': 28 | ctypes.UINT8,
    # H4 and H5 are 12bit shorts across 3 bytes with the LSB of both in 0xE5
    'H4_MSB': 29 | ctypes.INT8,
    'H4_LSB': 30 | ctypes.BFUINT8 | 0 << ctypes.BF_POS | 4 << ctypes.BF_LEN,
    'H5_LSB': 30 | ctypes.BFUINT8 | 4 << ctypes.BF_POS | 4 << ctypes.BF_LEN,
    'H5_MSB': 31 | ctypes.INT8,
    'H6': 32 | ctypes.INT8,
}


class BME280:
    def __init__(self, i2c, addr=DEV_ADDR):
        self.i2c = i2c
        self.addr = addr
        self._data_buf = bytearray(ctypes.sizeof(DATA, ctypes.BIG_ENDIAN))
        self.data = ctypes.struct(ctypes.addressof(self._data_buf), DATA,
                                  ctypes.BIG_ENDIAN)
        self._settings_buf = bytearray(ctypes.sizeof(SETTINGS,
                                                     ctypes.LITTLE_ENDIAN))
        self.settings = ctypes.struct(ctypes.addressof(self._settings_buf),
                                      SETTINGS, ctypes.LITTLE_ENDIAN)
        self._calibration_buf = bytearray(ctypes.sizeof(CALIBRATION,
                                                        ctypes.LITTLE_ENDIAN))
        self.calibration = ctypes.struct(
                            ctypes.addressof(self._calibration_buf),
                            CALIBRATION, ctypes.LITTLE_ENDIAN)
        self.t_fine = 0
        self.adc_p = 0
        self.adc_t = 0
        self.adc_h = 0
        self._load_calibration()
        self.read(force=False, settings=True)

    def _load_calibration(self):
        """Reads and stores factory set calibration data."""
        buf_ref = memoryview(self._calibration_buf)
        cal_a = buf_ref[:-7]
        cal_b = buf_ref[-7:]
        self.i2c.readfrom_mem_into(self.addr, ADDR_CALA, cal_a)
        self.i2c.readfrom_mem_into(self.addr, ADDR_CALB, cal_b)

    @property
    def temperature(self):
        """Calculation as set out in datasheet."""
        adc = self.data.adc_t
        c = self.calibration
        var1 = ((((adc >> 3) - (c.T1 << 1))) * c.T2) >> 11
        var2 = (((((adc >> 4) - c.T1) * ((adc >> 4) - c.T1)) >> 12) * c.T3) >> 14
        self.t_fine = var1 + var2
        return ((self.t_fine * 5 + 128) >> 8) / 100

    @property
    def pressure(self):
        """Calculation as set out in datasheet."""
        adc = self.data.adc_p
        c = self.calibration
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * c.P6
        var2 = var2 + ((var1*c.P5)<<17)
        var2 = var2 + (c.P4<<35)
        var1 = ((var1 * var1 * c.P3)>>8) + ((var1 * c.P2)<<12);
        var1 = (((1<<47)+var1))*(c.P1)>>33
        if var1 == 0:
            raise Exception("Ohno")
        p = 1048576-adc
        p = (((p<<31)-var2)*3125)//var1
        var1 = (c.P9 * (p>>13) * (p>>13)) >> 25
        var2 =(c.P8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (c.P7<<4)
        return p / 256

    @property
    def humidity(self):
        """Calculation as set out in datasheet."""
        adc = self.data.adc_h
        c = self.calibration
        H4 = c.H4_MSB << 4 | c.H4_LSB
        H5 = c.H5_MSB << 4 | c.H5_LSB
        v_x1_u32r = self.t_fine - 76800
        v_x1_u32r = ((((adc << 14) -(H4 << 20) - (H5 * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * c.H6) >> 10) * (((v_x1_u32r * c.H3) >> 11) + 32768)) >> 10) + 2097152) * c.H2 + 8192) >> 14)
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * c.H1) >> 4))
        return (v_x1_u32r>>12) / 1024

    def write_settings(self):
        waddr = ADDR_SETTINGS
        for _byte in self._settings_buf:
            self.i2c.writeto_mem(self.addr, waddr, bytearray((_byte,)))
            waddr += 1

    @property
    def delay(self):
        """Return maximum ms update delay.

        For forced mode this is the length of time to wait before reading the
        sample.

        For normal mode these values should be added to standby time.
        If normal is True standby time is added to the delay.

        This is essentially the max data rate
        i.e. (1000 / delay) = max data rate in Hz
        """
        t_oversample = OVERSAMPLE[self.settings.ctrl_meas.osrs_t]
        p_oversample = OVERSAMPLE[self.settings.ctrl_meas.osrs_p]
        h_oversample = OVERSAMPLE[self.settings.ctrl_hum.osrs_h]
        ms = t_oversample + p_oversample + h_oversample
        ms = (ms * 2.3) + 1.25
        if p_oversample > 0:
            ms += 0.575
        if h_oversample > 0:
            ms += 0.575
        if self.settings.ctrl_meas.mode == MODE_NORMAL:
            ms += STANDBY[self.settings.config.t_sb]
        return ms

    def read(self, force=True, settings=False, t_oversample=8):
        """Read current values from device registers.

        force -- instructs a sample to be taken (i.e. if auto sample off)
        settings -- Also read current device settings (always True with force)
        t_oversample -- Oversample setting for temperature in force mode

        """
        if settings or force:
            self.i2c.readfrom_mem_into(self.addr, ADDR_SETTINGS,
                                       self._settings_buf)
        if force:
            self.settings.ctrl_meas.mode = MODE_FORCED
            self.settings.ctrl_meas.osrs_t = OVERSAMPLE.index(t_oversample)
            self.settings.ctrl_meas.osrs_p = OVERSAMPLE.index(1)
            self.settings.ctrl_hum.osrs_h = OVERSAMPLE.index(1)
            self.write_settings()
            utime.sleep(self.delay / 1000)
        self.i2c.readfrom_mem_into(self.addr, ADDR_DATA, self._data_buf)
