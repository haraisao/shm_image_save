#
# Shared Memory Library
# Copyright(C) 2020 Isao Hara
#  License: Apache License v2.0
#  
import os
import sysv_ipc
import struct

#
#
class SharedMem(object):
  #
  def __init__(self, id=123456, create=None):
    if create :
      self.shm = sysv_ipc.SharedMemory(id, sysv_ipc.IPC_CREAT, mode=0666)
    else:
      self.shm = sysv_ipc.SharedMemory(id, 0, mode=0666)

    self.shm_offset={ 'top': 0 } 
  #
  def get_offset(self, name):
    if type(name) is int: return name
    else: return self.shm_offset[name] 

  #
  # helper functions to convert values
  def get_shm_bytes(self, name, size=2):
      return self.shm.read(size, self.get_offset(name))

  def get_char(self, off):
      return struct.unpack('c',self.get_shm_bytes(off, 1))[0]

  def get_uchar(self, off):
      return struct.unpack('B',self.get_shm_bytes(off, 1))[0]

  def get_short(self, off):
      return struct.unpack('h',self.get_shm_bytes(off, 2))[0]

  def get_ushort(self, off):
      return struct.unpack('H',self.get_shm_bytes(off, 2))[0]

  def get_int(self, off):
      return struct.unpack('i',self.get_shm_bytes(off, 4))[0]

  def get_uint(self, off):
      return struct.unpack('i',self.get_shm_bytes(off, 4))[0]

  def get_float(self, off):
      return struct.unpack('f',self.get_shm_bytes(off, 4))[0]

  def get_double(self, off):
      return struct.unpack('d',self.get_shm_bytes(off, 8))[0]
  ###
  #
  def set_char(self, name, val):
      self.shm.write( struct.pack('c',val),self.get_offset(name) )
      # for Python3
      #self.shm.write( struct.pack('c',bytes(val, 'utf-8')),self.get_offset(name) )
      return

  def set_uchar(self, name, val):
      self.shm.write( struct.pack('B',val), self.get_offset(name) )
      return

  def set_short(self, name, val):
      self.shm.write( struct.pack('h',val), self.get_offset(name) )
      return

  def set_ushort(self, name, val):
      self.shm.write( struct.pack('H', val), self.get_offset(name) )
      return

  def set_int(self, name, val):
      self.shm.write( struct.pack('i',val), self.get_offset(name) )
      return

  def set_uint(self, name, val):
      self.shm.write( struct.pack('I',val), self.get_offset(name) )
      return

  def set_float(self, name, val):
      self.shm.write( struct.pack('f',val), self.get_offset(name) )
      return

  def set_double(self, name, val):
      self.shm.write( struct.pack('d',val), self.get_offset(name) )
      return 

