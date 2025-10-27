class int32:
    def __new__(cls, u):
        return u - (1 << 32) if u & (1 << 31) else u