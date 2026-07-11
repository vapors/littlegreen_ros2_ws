# v2.6.1 validation record

Static source validation completed for the build-system hotfix:

- the driver core target is part of an install export set;
- the driver package calls `ament_export_targets(... HAS_LIBRARY_TARGET)`;
- all four maintenance executables link `lgh_st3215_driver::lgh_st3215_driver_core`;
- package dependency declarations remain present;
- XML, YAML, Python, and shell syntax checks from v2.6.0 remain applicable;
- no servo map, driver profile, runtime parameter, or policy file changed.

A real ROS 2 Humble build remains the authoritative integration check because the packaging environment does not contain the Orange Pi ROS overlay.
