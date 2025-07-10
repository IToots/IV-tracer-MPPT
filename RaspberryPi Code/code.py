# === Imports ===
import gc                 # Garbage collector for freeing memory
import math               # Math functions (not used here but imported)
import time               # Timing functions
import board              # Access to board pin definitions
import pwmio              # PWM output control
import busio              # I2C communication
import usb_cdc            # USB CDC communication over serial
import digitalio          # Digital I/O pins
import analogio           # Analog I/O pins (not used here)
from adafruit_ina219 import INA219, ADCResolution  # Current/voltage sensor
from adafruit_pcf8574 import PCF8574               # GPIO expander for LCD
from adafruit_character_lcd.character_lcd import Character_LCD_Mono  # LCD driver

"""
==========================
DISTANCE FROM LED 17-18CM!!
==========================
"""

# === Setup I2C and LCD ===
i2c_lcd = busio.I2C(scl=board.GP1, sda=board.GP0)
pcf = PCF8574(i2c_lcd, address=0x20)
lcd_rs = pcf.get_pin(0)
lcd_en = pcf.get_pin(2)
lcd_d4 = pcf.get_pin(4)
lcd_d5 = pcf.get_pin(5)
lcd_d6 = pcf.get_pin(6)
lcd_d7 = pcf.get_pin(7)
lcd_columns = 16
lcd_rows = 2
lcd = Character_LCD_Mono(lcd_rs, lcd_en,
                         lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                         lcd_columns, lcd_rows)

# === Setup INA219 sensor ===
i2c_ina = busio.I2C(board.GP7, board.GP6)
ina = INA219(i2c_ina)

# === Setup digital output to control the switch MOSFET (GP10) ===
mosfet_switch = digitalio.DigitalInOut(board.GP10)
mosfet_switch.direction = digitalio.Direction.OUTPUT

# === Initialize fs list ===
fs = []

# === Function to run IV tracing ===
def run():
    # Step 1: Turn switch MOSFET ON (parallel 0.1Ω + 1Ω load)
    mosfet_switch.value = True
    time.sleep(0.02)
    # Set high-current calibration (for coarse estimation)
    ina.set_calibration_32V_3A()
    ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_8S
    ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_8S
    time.sleep(0.01)
    # Temporary setup: Use GP9 to briefly test max current
    mosfet_9 = digitalio.DigitalInOut(board.GP9)
    mosfet_9.direction = digitalio.Direction.OUTPUT
    mosfet_9.value = True
    time.sleep(0.02)
    max_curr = ina.current  # Measure max current
    time.sleep(0.01)
    mosfet_9.value = False
    mosfet_9.deinit()  # Clean up pin to reuse later
    print(f"Max current estimate: {max_curr:.3f} A")
    # Decide which calibration to use based on measured current
    if max_curr >= 0.3:
        print("Current high - using 32V 3A calibration")
    else:
        mosfet_switch.value = False
        time.sleep(0.02)
        ina.set_calibration_32V_300mA()  # Better resolution for low current
        time.sleep(0.01)
        
    # === Loop through each test frequency ===
    for f in fs:
        print(f"\nTesting PWM frequency: {f} Hz")
        pwm = pwmio.PWMOut(board.GP9, frequency=f, duty_cycle=0)  # PWM pin
        comm.write(f"FREQ,{f}\n".encode())
        # === Step 1: Coarse PWM sweep from 0% to 100% in 1% steps ===
        ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_8S
        ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_8S
        time.sleep(0.02)
        duty_initial = None  # Start of current detection
        duty_final = None    # End of usable voltage range
        for duty in range(0, 101):  # 0% to 100% duty cycle
            duty_val = int((duty / 100) * 65535)
            pwm.duty_cycle = duty_val
            time.sleep(0.01)
            voltage = ina.bus_voltage + ina.shunt_voltage
            current = ina.current
            print(f"{duty},{voltage},{current}")
            if duty_initial is None and current > 0.001:
                duty_initial = duty
            if duty_final is None and voltage < 0.25:
                duty_final = duty
            power = voltage * current
        if duty_initial is None or duty_final is None:
            comm.write(b"FAILED\n")
            pwm.deinit()
            return
        comm.flush()
        
        # === Step 2: Fine PWM sweep in 0.05% increments ===
        ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_16S
        ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_16S
        time.sleep(0.02)
        start_duty = max(0, duty_initial - 5)
        end_duty = min(100, duty_final + 5)
        step_size = 0.05
        vmp = imp = pmax = 0.0  # Initialize max power tracking variables
        num_steps = int((end_duty - start_duty) / step_size) + 1
        for i in range(num_steps):
            duty = start_duty + i * step_size
            duty_val = int((duty / 100) * 65535)
            pwm.duty_cycle = duty_val
            time.sleep(0.05)
            voltage = ina.bus_voltage + ina.shunt_voltage
            current = ina.current
            voltage_corrected = voltage - 0.1 * current
            power = voltage * current
            power_corrected = voltage_corrected * current
            timestamp = time.monotonic()
            comm.write(f"{timestamp:.3f},{duty:.2f},{voltage:.3f},{voltage_corrected:.3f},{current:.3f},{power:.3f},{power_corrected:.3f}\n".encode())
            comm.flush()
            if power > pmax:
                vmp = voltage_corrected
                imp = current
                pmax = power_corrected
                
        # === Display results on LCD ===
        lcd.clear()
        lcd.message = f"Vmp:{vmp:.2f}V\nImp:{imp*1000:.1f}mA"
        time.sleep(2)
        lcd.clear()
        lcd.message = f"MPP:{pmax:.3f}W"
        pwm.deinit()
        gc.collect()

# === Function to run MPP tracking ===
def mppt_loop(step_size=0.05):
    global stop_mppt
    stop_mppt = False
    # Step 1: Turn switch MOSFET ON (parallel 0.1Ω + 1Ω load)
    mosfet_switch.value = True
    time.sleep(0.02)
    # Set high-current calibration (for coarse estimation)
    ina.set_calibration_32V_3A()
    ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_8S
    ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_8S
    time.sleep(0.01)
    # Temporary setup: Use GP9 to briefly test max current
    mosfet_9 = digitalio.DigitalInOut(board.GP9)
    mosfet_9.direction = digitalio.Direction.OUTPUT
    mosfet_9.value = True
    time.sleep(0.02)
    max_curr = ina.current  # Measure max current
    time.sleep(0.01)
    mosfet_9.value = False
    mosfet_9.deinit()  # Clean up pin to reuse later
    print(f"Max current estimate: {max_curr:.3f} A")
    # Decide which calibration to use based on measured current
    if max_curr >= 0.3:
        print("Current high - using 32V 3A calibration")
    else:
        mosfet_switch.value = False
        time.sleep(0.02)
        ina.set_calibration_32V_300mA()  # Better resolution for low current
        time.sleep(0.01)
    
    # Step 2: Coarse PWM sweep from 0% to 100% in 0.5% steps
    pwm = pwmio.PWMOut(board.GP9, frequency=fs[0], duty_cycle=0)
    comm.write(f"FREQ,{fs[0]}\n".encode())
        
    best_power = 0.0
    best_duty = 0
    for duty in [x * 0.5 for x in range(0, 201)]:
        pwm.duty_cycle = int((duty / 100) * 65535)
        time.sleep(0.01)
        voltage = ina.bus_voltage + ina.shunt_voltage
        current = ina.current
        power = voltage * current
        if power > best_power:
            best_power = power
            best_duty = duty
    
    # Step 3: MPP tracking
    ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_64S
    ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_64S
    duty = best_duty
    pwm.duty_cycle = int((duty / 100) * 65535)
    time.sleep(0.01)
    last_power = (ina.bus_voltage + ina.shunt_voltage) * ina.current
    direction = 1
    start_time = time.monotonic()
    # Data collection
    samples = []
    while True:
        if comm.in_waiting > 0:
                new_cmd = comm.readline().decode().strip().upper()
                if new_cmd == "STOP":
                    print("\n\nSTOP received inside mppt_loop")
                    stop_mppt = True
                    break
        for _ in range(10):  # 10 samples over 1 second
            voltage = ina.bus_voltage + ina.shunt_voltage
            current = ina.current
            voltage_corrected = voltage - 0.1 * current
            power = voltage * current
            power_corrected = voltage_corrected * current
            samples.append((voltage, voltage_corrected, current, power, power_corrected))
            time.sleep(0.1)
        # Compute averages
        avg_voltage = sum(v for v, _, _, _, _ in samples) / len(samples)
        avg_voltage_corrected = sum(vc for _, vc, _, _, _ in samples) / len(samples)
        avg_current = sum(c for _, _, c, _, _ in samples) / len(samples)
        avg_power = sum(p for _, _, _, p, _ in samples) / len(samples)
        avg_power_corrected = sum(pc for _, _, _, _, pc in samples) / len(samples)
        samples.clear()
        # Serial output
        timestamp = time.monotonic() - start_time
        comm.write(f"{timestamp:.1f},{duty:.2f},{avg_voltage:.3f},{avg_voltage_corrected:.3f},{avg_current:.3f},{avg_power:.3f},{avg_power_corrected:.3f}\n".encode())
        print(f"{timestamp:.1f},{avg_power_corrected:.3f}")

        # LCD output
        lcd.clear()
        lcd.message = f"P:{avg_power_corrected:.3f}W"

        # Perturb & Observe algorithm
        if avg_power > last_power:
            last_power = avg_power
        else:
            direction *= -1
        # Update duty cycle
        duty += direction * step_size
        duty = max(0, min(100, duty))
        pwm.duty_cycle = int((duty / 100) * 65535)
        
        ina.bus_adc_resolution = ADCResolution.ADCRES_12BIT_64S
        ina.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_64S

        duty = best_duty
        direction = 1
        start_time = time.monotonic()
        last_power = 0

        def measure_power(duty_value):
            pwm.duty_cycle = int((duty_value / 100) * 65535)
            time.sleep(0.1)  # let system settle
            power_samples = []
            power_corrected_samples = []
            for _ in range(10):
                voltage = ina.bus_voltage + ina.shunt_voltage
                current = ina.current
                voltage_corrected = voltage - 0.1 * current
                power = voltage * current
                power_corrected = voltage_corrected * current
                power_samples.append(power)
                power_corrected_samples.append(power_corrected)
                time.sleep(0.01)
            avg_power = sum(power_samples) / len(power_samples)
            avg_power_corrected = sum(power_corrected_samples) / len(power_corrected_samples)
            return avg_power, avg_power_corrected, voltage, voltage_corrected, current

        while not stop_mppt:
            if comm.in_waiting > 0:
                new_cmd = comm.readline().decode().strip().upper()
                print(new_cmd)
                if new_cmd == "STOP":
                    print("\n\nSTOP received inside mppt_loop")
                    stop_mppt = True
                    break
            # Measure all three duty points (center, plus, minus)
            p_center, pc_center, v_center, vc_center, i_center = measure_power(duty)
            p_plus, pc_plus, _, _, _ = measure_power(min(duty + step_size, 100))
            p_minus, pc_minus, _, _, _ = measure_power(max(duty - step_size, 0))

            timestamp = time.monotonic() - start_time

            # Send both average power and corrected power for the center duty cycle
            comm.write(f"{timestamp:.1f},{duty:.2f},{p_center:.3f},{pc_center:.3f}\n".encode())
            print(f"{timestamp:.1f}, Duty: {duty:.2f}, Power: {p_center:.3f}, Corrected: {pc_center:.3f}")
            print(f"Voltage: {v_center}, Corrected: {vc_center}, Current: {i_center}")

            lcd.clear()
            lcd.message = f"P:{pc_center:.3f}W"

            # Use uncorrected power (or corrected if you prefer) for P&O decision
            if p_plus > p_center and p_plus > p_minus:
                direction = 1
                last_power = p_plus
                duty = min(duty + step_size, 100)
            elif p_minus > p_center and p_minus > p_plus:
                direction = -1
                last_power = p_minus
                duty = max(duty - step_size, 0)
            else:
                if p_center > last_power:
                    last_power = p_center
                else:
                    direction *= -1
                duty = max(0, min(100, duty + direction * step_size))

            pwm.duty_cycle = int((duty / 100) * 65535)
        
        # Cleanup
        pwm.duty_cycle = 0
        pwm.deinit()
        mosfet_switch.value = False
        lcd.clear()
        lcd.message = "MPPT Done"
        comm.write(b"MPPT_DONE\n")
            
# === Main loop: Wait for USB command to trigger measurement ===
comm = usb_cdc.data
print("Ready to receive command")
while True:
    if comm.in_waiting > 0:
        raw = comm.readline()
        command = raw.decode('utf-8').strip().upper()
        print(f"Received: {command}")
        if command == "SWEEP":
            # === PWM frequencies to sweep through (in Hz) ===
            fs[:] = [90_000, 85_000, 80_000, 75_000, 70_000, 65_000, 60_000]
            time.sleep(1)
            run()
            time.sleep(1)
            comm.write(b"END\n")
        elif command == "SINGLE":
            # === PWM frequency (in Hz) ===
            fs[:] = [81_000]
            time.sleep(1)
            run()
            time.sleep(1)
            comm.write(b"END\n")
        elif command == "MPPT":
            stop_mppt = False
            fs[:] = [81000]
            lcd.clear()
            lcd.message = "MPPT Starting..."
            time.sleep(1)
            mppt_loop()
        elif command == "STOP":
            print("STOP command received")
            stop_mppt = True
