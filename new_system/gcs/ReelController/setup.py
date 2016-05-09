from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

extensions = [
    Extension("rc", 
    sources=["PyReelController.pyx", "EposMotorController.cpp", "ReelController.cpp"],
    language="c++", 
    include_dirs = ["include"],
    libraries = ["EposCmd"],
    library_dirs = ["lib"],)
]

setup(ext_modules = cythonize(extensions))
      