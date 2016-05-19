from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

extensions = [
    Extension("PyMotorController", 
    sources=["PyMotorController.pyx", "EposMotorController.cpp"],
    language="c++", 
    include_dirs = ["include"],
    libraries = ["EposCmd"],
    library_dirs = ["lib"],)
]

setup(ext_modules = cythonize(extensions))
      
