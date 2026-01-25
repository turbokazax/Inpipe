from Logic.Telemetry import Sender, Sniffer, PortManager, getPorts
# from Routines import test933

import time, os
filename = os.path.basename(__file__).strip(".py")
print(filename)
# telemetry = Sender()
# print(telemetry.getPort())
# sniffer = Sniffer(port = telemetry.getPort())
sender = Sender(port = 10000, name = "MotorData1")
sniffer = Sniffer(port = 10000)
# sender2 = Sender(port = 10001, name = "MotorData2")
# print(getPorts())
# SimpleTelemetry.add(1,2,None, None)
c = 0
while True:
    # telemetry.add("123", "456")
    # telemetry.update()
    # test933.test933.run()
    # print(sniffer.drain_udp())
    # pkt = sniffer.drain_udp()
    # if pkt is not None:
        # print(pkt)
    # time.sleep(0.01)
    # print(SimpleTelemetry.toString())
    sender.add("1", "2", "3")
    sender.update()
    sniffer.log_CSV()
    # sender2.update()
    # ports = getPorts()
    # if ports is not None and c < 1:
    #     print(f"{len(ports)} ports available: {ports}")
    #     c+=1