from distutils.core import setup
from Cython.Build import cythonize

extensions = [
    "I2cSensor.py",
    "AllSensors.py",
    "Microchip.py",
    "Adafruit_I2C.py",
]

setup(
    ext_modules = cythonize(extensions)
)
