import socket, time, errno,random
from pathlib import Path
import re

from Misc.Helpers import getDateTime

class UDPManager():
    def __init__(self, ip = "127.0.0.1"):
        self.ip = ip
        self._portList = []
    
    def _is_port_busy(self, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setblocking(False)
        try:
            s.bind((self.ip, port))
            return False
        except OSError as e:
            if e.errno == errno.EADDRINUSE:
                return True
            else:
                raise
        finally:
            s.close()
    
    def getLocalhost(self):
        return "127.0.0.1"

    def getIP(self):
        return self.ip

    def getFreePort(self):
        _port_IDs = range(49152, 65535)
        _last_tested_Ports = []
        while True:
            _port = random.randint(_port_IDs[0], _port_IDs[-1])
            if _port in _last_tested_Ports:
                continue
            if not self._is_port_busy(_port):
                print(f"Found free port - {_port}")
                return _port
            else:
                _last_tested_Ports.append(_port)
                if len(_last_tested_Ports) > 100:
                    _last_tested_Ports.pop(0)

udpman = UDPManager()

class Sender():
    def __init__(self, ip = "127.0.0.1", port = None, freq = 1/100, name = "Untitlaaed"):
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.viewer_addr = (ip, port)
        self.port = port if port is not None else udpman.getFreePort()
        # if port is None or self.port is None:
        #     self.port = udpman.getFreePort()
        # else:
        #     self.port = port
        self.ip = ip if ip is not None else udpman.getLocalhost()
        self.viewer_addr = (self.ip, self.port)
        self._now = 0
        self._last_sent = 0
        self.name = name
        self._send_period = freq # in Seconds (e.g. freq = 1/100 is 100 Hz)
        self.msg = ""
        self._prev_msg = ""
        print(f"TELEMETRY_{self.name}: Sender instantiated at Host = {self.ip}, Port = {self.port}")
        self._notify_master()

    def add(self, x):
        self.msg += f"{x},"
    
    def add(self, *x):
        # for _obj in x:
            # self.msg += f"{_obj},"
        self.msg = ",".join(str(v) for v in x)

    def getPort(self):
        return self.port

    def _notify_master(self):
        try:
            payload = f"TELEMETRY_{self.name},{self.port}".encode()
            _master_viewer_addr = (self.ip, 9998)
            self.udp.sendto(payload, _master_viewer_addr)
        except OSError as e:
            print(f"TELEMETRY: Failed to notify master, {repr(e)}, {getattr(e, "errno", None)}")

    def getName(self):
        return self.name

    def update(self):
        self._now = time.time()
        if self._now - self._last_sent >= self._send_period:
            self._notify_master()
            try:
                payload = self.msg.strip(" ,").encode()
                _payload = payload.decode()
                if len(_payload) > 0:
                    self.udp.sendto(payload, self.viewer_addr)
                self._prev_msg = payload
                self.msg = ""
                self._last_sent = self._now  # <-- add this
            except OSError as e:
                print("TELEMETRY: Failed to send message.", repr(e), "errno=", getattr(e, "errno", None))
                self.msg = ""   # drop backlog

    
    def toString(self):
        return f"Host - {self.ip}, Port - {self.port}, Message - {self._prev_msg.decode()}"

# SimpleTelemetry = SenderTelemetry()

class Sniffer():
    def __init__(self, host="127.0.0.1", port=9999):
        self.ip = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.ip, self.port))
        self.socket.setblocking(False)
        self.buff_size = 2048
        self.log_directory = "./Logs/"
        self._active_logfiles = {}  

    def drain_udp(self, max_packets=64, split = True):
        """
        Drain up to max_packets queued datagrams and return the newest split packet.
        Returns None if nothing is available (normal).
        """
        last = None
        for _ in range(max_packets):
            try:
                data, _addr = self.socket.recvfrom(self.buff_size)
            except BlockingIOError:
                break  # normal: nothing available

            s = data.decode(errors="ignore").strip()
            if not s:
                continue
            if split:
                last = s.split(",")
            else:
                last = s
        return last

    def log_CSV(self, name="untitled"):
        """
        Create logfile name ONCE per run (per Sniffer instance), then append forever.
        If Logs/<name>.csv exists, choose Logs/<name>_<N+1>_<datetime>.csv.
        """
        log_dir = Path(self.log_directory)
        log_dir.mkdir(parents=True, exist_ok=True)

        base = str(name).strip()

        # Reuse chosen file for this run
        if base in self._active_logfiles:
            out_file = self._active_logfiles[base]
        else:
            base_file = log_dir / f"{base}.csv"

            if not base_file.exists():
                out_file = base_file
            else:
                # Find max existing suffix among base_#.csv AND base_#_<datetime>.csv
                # We only care about the number part.
                pat = re.compile(rf"^{re.escape(base)}_(\d+)(?:_.*)?\.csv$")
                max_n = 0
                for p in log_dir.iterdir():
                    if not p.is_file():
                        continue
                    m = pat.match(p.name)
                    if m:
                        max_n = max(max_n, int(m.group(1)))

                dt = getDateTime()  # your helper, used once per run
                out_file = log_dir / f"{base}_{max_n + 1}_{dt}.csv"

                # Extra safety: if dt format collides, bump until free
                while out_file.exists():
                    max_n += 1
                    out_file = log_dir / f"{base}_{max_n + 1}_{dt}.csv"

            self._active_logfiles[base] = out_file

        msg = self.drain_udp(split=False)
        if msg is not None:
            with out_file.open("a", encoding="utf-8") as f:
                print(msg, file=f)





# PortManager = Sniffer(port = 9990) # SHIT NOBODY CARES ABOUT

# def getPorts():
#     ports = PortManager.drain_udp()
#     # return ports if ports is not None else "NONE"
#     return ports

# _internal = SenderTelemetry()
    
# class Sniffer():
    