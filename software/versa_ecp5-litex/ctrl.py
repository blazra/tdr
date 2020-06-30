#!/usr/bin/env python3

from litex import RemoteClient
import matplotlib.pyplot as plt
import numpy as np

def bin_array(num, m):
    """Convert a positive integer num into an m-bit bit vector"""
    return np.array(list(np.binary_repr(num).zfill(m))).astype(np.int8)

def vec_bin_array(arr, m):
    """
    Arguments: 
    arr: Numpy array of positive integers
    m: Number of bits of each integer to retain

    Returns a copy of arr with every element replaced with a bit vector.
    Bits encoded as int8's.
    """
    to_str_func = np.vectorize(lambda x: np.binary_repr(x).zfill(m))
    strs = to_str_func(arr)
    ret = np.zeros(list(arr.shape) + [m], dtype=np.int8)
    for bit_ix in range(0, m):
        fetch_bit_func = np.vectorize(lambda x: x[bit_ix] == '1')
        ret[...,bit_ix] = fetch_bit_func(strs).astype("int8")

    return ret 


length = 25
result = np.empty((0, length*20))

wb = RemoteClient()
wb.open()
wb.debug = True

for v in range(0x3800, 0x3FFF, 1):
    assert v >= 0x3000
    assert v <= 0x3FFF
    wb.regs.spidac_mosi.write(v)
    wb.regs.spidac_control.write(0x1001)
    wb.regs.daq_trigger.write(0)
    wb.regs.daq_trigger.write(1)

    mem = wb.read(wb.mems.daq_memory.base, length=length)
    mem = np.array(mem)
    mem = np.fliplr(vec_bin_array(mem, 20))
    mem = np.reshape(mem, (1,length*20))

    result = np.append(result, mem, axis=0)


plt.matshow(result, origin='lower', aspect='auto')
plt.show()
np.save("data", result)


[...]


wb.close()

