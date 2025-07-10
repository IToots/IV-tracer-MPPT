import board
import busio

i2c = busio.I2C(scl=board.GP1, sda=board.GP0)
while not i2c.try_lock():
    pass
print("I2C addresses found:", [hex(addr) for addr in i2c.scan()])
i2c.unlock()

i2c = busio.I2C(scl=board.GP7, sda=board.GP6)
while not i2c.try_lock():
    pass
print("I2C addresses found:", [hex(addr) for addr in i2c.scan()])
i2c.unlock()
