# Archived ZeroErr Torque Sensor Probe

`zeroerr_torque_sensor_probe.py` is a standalone ROS 2 diagnostic tool that
queries `ethercat_manager/get_sdo` for each ZeroErr slave.

It reads:
- `0x2241:00` dual encoder difference
- `0x3B69:00` virtual torque sensor value in mNm
- `0x3B6A:00` torque sensor ratio

The script was not referenced by active launch files, runtime code, shell
scripts, or configs. It was only installed as a manually runnable executable, so
it has been archived to keep the active package surface smaller.
