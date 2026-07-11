# lgh_st3215_maintenance

Offline, read-only ST3215 maintenance commands. These executables open `/dev/ttyS3` directly and therefore require `lgh_st3215_driver` to be stopped.

The shared `SerialPort` implementation acquires `/tmp/lgh_st3215_dev_ttyS3.lock` (device name sanitized) before opening the UART. If the runtime driver owns the port, maintenance refuses with exit code **3**.

```bash
ros2 run lgh_st3215_maintenance bus_scan
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

v2.6.0 intentionally contains no ID changes, baud changes, EEPROM writes, or factory reset command.
