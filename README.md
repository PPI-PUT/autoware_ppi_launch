# autoware_ppi_launch

## Structure

![autoware_launch](./overall-node-diagram-autoware-universe.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

### Autoware main
You can use the command as follows at shell script to launch `*.launch.xml` in `launch` directory.

```bash
ros2 launch autoware_ppi_launch apm.launch.xml
```

### Mapping
```bash
ros2 launch autoware_ppi_launch lidarslam.launch.py
```

To save the map, use the following command:
```bash
ros2 service call /map_save std_srvs/Empty
```
