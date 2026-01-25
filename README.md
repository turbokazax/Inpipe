# ----------------- TODO --------------------
- Add variable timing to CSV logging, so it could be sync-d with the Optitrack position outputs

# ----------------- DONE --------------------
- DCMotor: all EEPROM variables controlling methods implemented
- Bulk reading/writing saves a lot of compute & time (iter. time is less than 0.0001s compared to ~0.01s before)
- Four-motor setup: adjacent pairs of motors (theoretically, yet to test) rotate the end effector quadrant-by-quadrant (also, with shrinking radius by dt) 
- Telemetry: Sending, Sniffing (Receiving) via UDP Packets

