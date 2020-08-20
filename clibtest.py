import ctypes

cdlib = ctypes.CDLL("/home/pi/projects/ctypes/libsapphire.so")

rc = cdlib.dlogin(5)
rc = cdlib.begin()
print(rc)
str = b"filename"
rc = cdlib.gstring(ctypes.c_char_p(str))
str = "database"
rc = cdlib.gstring(ctypes.c_char_p(str.encode()))
cdlib.getbytes.restype = ctypes.c_char_p
pstr=cdlib.getbytes(0)
print(pstr)
print(pstr.decode())
