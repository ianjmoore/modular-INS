# modular-INS
Implementation of an externally aided tactical grade INS with Dual-Antenna GNSS receiver and RTK corrections via a one-shot launch file.
<br/>
## Bill of Materials:
### Hardware
#### INS:
- MicroStrain [CV7-INS](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-navigation-systems--ins-/3dm-cv7-ins)
- [Black c-series dev board](https://www.hbkworld.com/en/products/transducers/inertial-sensors/inertial-sensors-accessories/p-6212-3009) 
#### GNSS:
- Septentrio [mosaic go heading evaluation kit](https://www.septentrio.com/en/products/gnss-receivers/gnss-receiver-modules/mosaic-h-evaluation-kit) (Mosaic H receiver)
- SMA - TNC GNSS connector cable
- 2x L1/L2 or Full Band GNSS Antennas
- [SensorCloudRTK](https://rtkapp.sensorcloud.com/) API
#### Compute:
- Linux machine running Ubuntu 18.04 or later
### Software:
- [microstrain_inertial](https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros2) ROS 2 driver
- [septentrio_gnss_driver](https://github.com/septentrio-gnss/septentrio_gnss_driver) ROS 2 driver
- [ntrip_client](https://github.com/LORD-MicroStrain/ntrip_client/tree/ros2) ROS 2 driver
