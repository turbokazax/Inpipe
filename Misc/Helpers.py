def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def int32(u):
        return u - (1 << 32) if u & (1 << 31) else u

from datetime import datetime

def getTime():
      time = datetime.now().strftime("%H:%M:%S.%f")
      return time

def getDateTime():
      dt = datetime.now().strftime("%y-%m-%d_%H-%M-%S")
      return dt