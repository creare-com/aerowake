import time
import struct

num_tests = int(1e6)

def bytes_to_bitstring(bytes):
    bits = ''.join(['0'*(8-byte.bit_length())+ bin(byte)[2:] for byte in bytes])
    return bits

def extract_string_method(bytes):
    bits = ''.join(['0'*(8-byte.bit_length())+ bin(byte)[2:] for byte in bytes])
    #bits = bytes_to_bitstring(bytes)
    return int(bits[2:16], 2)

def extract_struct_method(bytes):
    """ Only works with 2 bytes """
    num=struct.unpack('H',struct.pack('BB',bytes[1],bytes[0]))[0]
    return num & 0x3FFF

data=[5,7]

t0 = time.time()
for i in xrange(0, num_tests):
    val=extract_string_method(data)
t1 = time.time()

string_method_avg_time = (t1-t0) / num_tests

t0 = time.time()
for i in xrange(0, num_tests):
    val=extract_struct_method(data)
t1 = time.time()

struct_method_avg_time = (t1-t0) / num_tests

print "String method: %fus avg.  Struct method: %fus avg."%(string_method_avg_time * 1e6, struct_method_avg_time *1e6)

