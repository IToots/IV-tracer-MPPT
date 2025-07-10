# SPDX-FileCopyrightText: 2017 Dean Miller for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_ina219`
====================================================

CircuitPython driver for the INA219 current sensor.

* Author(s): Dean Miller

Implementation Notes
--------------------

**Hardware:**

* `Adafruit INA219 High Side DC Current Sensor Breakout <https://www.adafruit.com/product/904>`_

* `Adafruit INA219 FeatherWing <https://www.adafruit.com/product/3650>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware (2.2.0+) for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from micropython import const

try:
    import typing

    from busio import I2C
except ImportError:
    pass

__version__ = "3.4.27"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_INA219.git"

# Bits

# Config Register (R/W)
_REG_CONFIG = const(0x00)


class BusVoltageRange:
    """Constants for ``bus_voltage_range``"""

    RANGE_16V = 0x00  # set bus voltage range to 16V
    RANGE_32V = 0x01  # set bus voltage range to 32V (default)


class Gain:
    """Constants for ``gain``"""

    DIV_1_40MV = 0x00  # shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV = 0x01  # shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV = 0x02  # shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV = 0x03  # shunt prog. gain set to /8, 320 mV range


class ADCResolution:
    """Constants for ``bus_adc_resolution`` or ``shunt_adc_resolution``"""

    ADCRES_9BIT_1S = 0x00  #  9bit,   1 sample,     84us
    ADCRES_10BIT_1S = 0x01  # 10bit,   1 sample,    148us
    ADCRES_11BIT_1S = 0x02  # 11 bit,  1 sample,    276us
    ADCRES_12BIT_1S = 0x03  # 12 bit,  1 sample,    532us
    ADCRES_12BIT_2S = 0x09  # 12 bit,  2 samples,  1.06ms
    ADCRES_12BIT_4S = 0x0A  # 12 bit,  4 samples,  2.13ms
    ADCRES_12BIT_8S = 0x0B  # 12bit,   8 samples,  4.26ms
    ADCRES_12BIT_16S = 0x0C  # 12bit,  16 samples,  8.51ms
    ADCRES_12BIT_32S = 0x0D  # 12bit,  32 samples, 17.02ms
    ADCRES_12BIT_64S = 0x0E  # 12bit,  64 samples, 34.05ms
    ADCRES_12BIT_128S = 0x0F  # 12bit, 128 samples, 68.10ms


class Mode:
    """Constants for ``mode``"""

    POWERDOWN = 0x00  # power down
    SVOLT_TRIGGERED = 0x01  # shunt voltage triggered
    BVOLT_TRIGGERED = 0x02  # bus voltage triggered
    SANDBVOLT_TRIGGERED = 0x03  # shunt and bus voltage triggered
    ADCOFF = 0x04  # ADC off
    SVOLT_CONTINUOUS = 0x05  # shunt voltage continuous
    BVOLT_CONTINUOUS = 0x06  # bus voltage continuous
    SANDBVOLT_CONTINUOUS = 0x07  # shunt and bus voltage continuous


# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE = const(0x01)

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE = const(0x02)

# POWER REGISTER (R)
_REG_POWER = const(0x03)

# CURRENT REGISTER (R)
_REG_CURRENT = const(0x04)

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION = const(0x05)
# pylint: enable=too-few-public-methods


def _to_signed(num: int) -> int:
    if num > 0x7FFF:
        num -= 0x10000
    return num


class INA219:
    """Driver for the INA219 current sensor"""

    # Basic API:

    # INA219( i2c_bus, addr)  Create instance of INA219 sensor
    #    :param i2c_bus          The I2C bus the INA219is connected to
    #    :param addr (0x40)      Address of the INA219 on the bus (default 0x40)

    # shunt_voltage               RO : shunt voltage scaled to Volts
    # bus_voltage                 RO : bus voltage (V- to GND) scaled to volts (==load voltage)
    # current                     RO : current through shunt, scaled to mA
    # power                       RO : power consumption of the load, scaled to Watt
    # set_calibration_32V_2A()    Initialize chip for 32V max and up to 2A (default)
    # set_calibration_32V_1A()    Initialize chip for 32V max and up to 1A
    # set_calibration_16V_400mA() Initialize chip for 16V max and up to 400mA

    # Advanced API:
    # config register break-up
    #   reset                     WO : Write Reset.RESET to reset the chip (must recalibrate)
    #   bus_voltage_range         RW : Bus Voltage Range field (use BusVoltageRange.XXX constants)
    #   gain                      RW : Programmable Gain field (use Gain.XXX constants)
    #   bus_adc_resolution        RW : Bus ADC resolution and averaging modes (ADCResolution.XXX)
    #   shunt_adc_resolution      RW : Shunt ADC resolution and averaging modes (ADCResolution.XXX)
    #   mode                      RW : operating modes in config register (use Mode.XXX constants)

    # raw_shunt_voltage           RO : Shunt Voltage register (not scaled)
    # raw_bus_voltage             RO : Bus Voltage field in Bus Voltage register (not scaled)
    # conversion_ready            RO : Conversion Ready bit in Bus Voltage register
    # overflow                    RO : Math Overflow bit in Bus Voltage register
    # raw_power                   RO : Power register (not scaled)
    # raw_current                 RO : Current register (not scaled)
    # calibration                 RW : calibration register (note: value is cached)

    def __init__(self, i2c_bus: I2C, addr: int = 0x40) -> None:
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.i2c_addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.set_calibration_32V_3A()

    # config register break-up
    reset = RWBits(1, _REG_CONFIG, 15, 2, False)
    bus_voltage_range = RWBits(1, _REG_CONFIG, 13, 2, False)
    gain = RWBits(2, _REG_CONFIG, 11, 2, False)
    bus_adc_resolution = RWBits(4, _REG_CONFIG, 7, 2, False)
    shunt_adc_resolution = RWBits(4, _REG_CONFIG, 3, 2, False)
    mode = RWBits(3, _REG_CONFIG, 0, 2, False)

    # shunt voltage register
    raw_shunt_voltage = ROUnaryStruct(_REG_SHUNTVOLTAGE, ">h")

    # bus voltage register
    raw_bus_voltage = ROBits(13, _REG_BUSVOLTAGE, 3, 2, False)
    conversion_ready = ROBit(_REG_BUSVOLTAGE, 1, 2, False)
    overflow = ROBit(_REG_BUSVOLTAGE, 0, 2, False)

    # power and current registers
    raw_power = ROUnaryStruct(_REG_POWER, ">H")
    raw_current = ROUnaryStruct(_REG_CURRENT, ">h")

    # calibration register
    _raw_calibration = UnaryStruct(_REG_CALIBRATION, ">H")

    @property
    def calibration(self) -> int:
        """Calibration register (cached value)"""
        return self._cal_value  # return cached value

    @calibration.setter
    def calibration(self, cal_value: int) -> None:
        self._cal_value = cal_value  # value is cached for ``current`` and ``power`` properties
        self._raw_calibration = self._cal_value

    @property
    def shunt_voltage(self) -> float:
        """The shunt voltage (between V+ and V-) in Volts (so +-.327V)"""
        # The least signficant bit is 10uV which is 0.00001 volts
        return self.raw_shunt_voltage * 0.00001

    @property
    def bus_voltage(self) -> float:
        """The bus voltage (between V- and GND) in Volts"""
        # Shift to the right 3 to drop CNVR and OVF and multiply by LSB
        # Each least signficant bit is 4mV
        return self.raw_bus_voltage * 0.004

    @property
    def current(self) -> float:
        """The current through the shunt resistor in milliamps."""
        # Sometimes a sharp load will reset the INA219, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... always setting a cal
        # value even if it's an unfortunate extra step
        self._raw_calibration = self._cal_value
        # Now we can safely read the CURRENT register!
        return self.raw_current * self._current_lsb

    @property
    def power(self) -> float:
        """The power through the load in Watt."""
        # Sometimes a sharp load will reset the INA219, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... always setting a cal
        # value even if it's an unfortunate extra step
        self._raw_calibration = self._cal_value
        # Now we can safely read the CURRENT register!
        return self.raw_power * self._power_lsb

    def set_calibration_32V_3A(self) -> None:
        """Calibration for max 3A, 32V."""
        self._current_lsb = 0.003  # 3 mA per bit
        self._cal_value = int(0.04096 / (self._current_lsb * 0.122))
        self._power_lsb = self._current_lsb * 20  # Power LSB = 60 mW/bit

        self.calibration = self._cal_value

        # Config register settings
        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV  # ±320mV across shunt supports <= 3 A
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
    
    def set_calibration_32V_300mA(self) -> None:
        """Calibration for max 300mA 32V"""
        self._current_lsb = 0.0003  # 300uA per bit
        self._cal_value = int(0.04096 / (self._current_lsb * 0.9835))
        self._power_lsb = self._current_lsb * 20

        self.calibration = self._cal_value

        # Set Config values
        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV  # ±320mV across shunt supports <= 300 mA
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS

