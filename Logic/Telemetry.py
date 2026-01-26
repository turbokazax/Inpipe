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

import socket
import re
from pathlib import Path

# You already have this helper somewhere
# def getDateTime() -> str: ...


class Sniffer:
    """
    UDP telemetry sniffer that logs newest packet to a CSV.
    - Creates a logfile name ONCE per run (per Sniffer instance), then appends forever.
    - If Logs/<name>.csv exists, uses Logs/<name>_<N+1>_<datetime>.csv.
    - Writes header once per file (only if file is empty).
    - Keeps per-file line counters in memory (fast; no re-reading file).
    """

    def __init__(self, host="127.0.0.1", port=9999, csv_header=None, log_directory="./Logs/"):
        self.ip = host
        self.port = port

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.ip, self.port))
        self.socket.setblocking(False)

        self.buff_size = 2048
        self.log_directory = str(log_directory)

        # base -> Path for this run
        self._active_logfiles: dict[str, Path] = {}

        # base -> next id to write (starts at 1)
        self._csv_line: dict[str, int] = {}

        # base values we've already written header for (this run)
        self._header_written: set[str] = set()

        self.csv_header = csv_header

        print(f"TELEMETRY_SNIFFER: Listening on {self.ip}:{self.port}")

    def set_csv_header(self, header: str | None):
        self.csv_header = header

    def drain_udp(self, max_packets=64, split=True):
        """
        Drain up to max_packets queued datagrams and return the newest packet.
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

            last = s.split(",") if split else s

        return last

    def _choose_out_file(self, base: str) -> Path:
        """
        Choose and cache a logfile path for this base (once per run).
        """
        if base in self._active_logfiles:
            return self._active_logfiles[base]

        log_dir = Path(self.log_directory)
        log_dir.mkdir(parents=True, exist_ok=True)

        base_file = log_dir / f"{base}.csv"
        if not base_file.exists():
            out_file = base_file
        else:
            # Find max existing suffix among base_#.csv AND base_#_<datetime>.csv
            pat = re.compile(rf"^{re.escape(base)}_(\d+)(?:_.*)?\.csv$")
            max_n = 0
            for p in log_dir.iterdir():
                if not p.is_file():
                    continue
                m = pat.match(p.name)
                if m:
                    max_n = max(max_n, int(m.group(1)))

            dt = getDateTime()  # used once per run
            out_file = log_dir / f"{base}_{max_n + 1}_{dt}.csv"

            # Extra safety: if dt format collides, bump until free
            while out_file.exists():
                max_n += 1
                out_file = log_dir / f"{base}_{max_n + 1}_{dt}.csv"

        self._active_logfiles[base] = out_file
        return out_file

    def _ensure_header(self, base: str, out_file: Path):
        """
        Write CSV header once per base file, only if the file is empty.
        """
        if self.csv_header is None:
            return
        if base in self._header_written:
            return

        try:
            # Only write header if file is empty (also works if file doesn't exist yet)
            if not out_file.exists() or out_file.stat().st_size == 0:
                with out_file.open("a", encoding="utf-8") as f:
                    # print(self.csv_header, file=f)
                    print("ID," + self.csv_header.strip(" ,"), file=f)
        except OSError as e:
            print(f"TELEMETRY_SNIFFER: Failed to write header to {out_file}: {repr(e)}")
            return

        self._header_written.add(base)

    def log_CSV(self, name="untitled"):
        """
        Drain UDP and append newest packet to CSV.
        IDs start at 1 (per file) and increment only when a row is actually written.
        """
        base = str(name).strip() or "untitled"
        out_file = self._choose_out_file(base)

        # Initialize counter for this file (IDs start at 1)
        if base not in self._csv_line:
            self._csv_line[base] = 1

        msg = self.drain_udp(split=False)
        if msg is None:
            return  # nothing to log

        # Ensure header is present before first data row
        self._ensure_header(base, out_file)

        line_id = self._csv_line[base]
        row = f"{line_id},{msg}"

        try:
            with out_file.open("a", encoding="utf-8") as f:
                print(row, file=f)
            self._csv_line[base] += 1
        except OSError as e:
            print(f"TELEMETRY_SNIFFER: Failed to write to log file {out_file}: {repr(e)}")




# PortManager = Sniffer(port = 9990) # SHIT NOBODY CARES ABOUT

# def getPorts():
#     ports = PortManager.drain_udp()
#     # return ports if ports is not None else "NONE"
#     return ports

# _internal = SenderTelemetry()
    
# class Sniffer():
    