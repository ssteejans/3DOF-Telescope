# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 10:01:18 2017

@author: mecha31
"""

def GetTwosComplement(value, bits):
    if (value & (1<<(bits-1))) != 0:
        value = value - (1<<bits)
    return value
    


