import time
import board
import digitalio
import busio
from adafruit_ina219 import INA219, ADCResolution

# === Setup I2C and INA219 ===
i2c_ina = busio.I2C(board.GP7, board.GP6)
ina = INA219(i2c_ina)

# === Function to estimate max current ===
def max_current_estimate():
    mosfet_9 = digitalio.DigitalInOut(board.GP9)
    mosfet_9.direction = digitalio.Direction.OUTPUT
    # === Setup digital output to control the switch MOSFET (GP10) ===
    mosfet_switch = digitalio.DigitalInOut(board.GP10)
    mosfet_switch.direction = digitalio.Direction.OUTPUT
    
    mosfet_switch.value = True
    # Turn on MOSFET briefly
    mosfet_9.value = True
    time.sleep(0.02)
    current = ina.current
    time.sleep(0.01)
    mosfet_9.value = False
    
    mosfet_9.deinit()
    mosfet_switch.deinit()

    return current

# === Main loop ===
print("Starting max current monitoring every 1 second...")

ina.set_calibration_32V_3A()
# ina.set_calibration_32V_300mA()

ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_8S
ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_8S

while True:
    max_curr = max_current_estimate()
    print(f"Max current estimate: {max_curr:.3f} A")
    time.sleep(1)
