# JetAuto navigation transport example

This workspace carries a single `jetauto_example` scenario: drive to a pick station, run the automatic pick routine, then drive to a drop station and drop off the object.


## Files to edit
To get started, edit the following files in the "jetauto_ws":

- `src/jetauto_example/scripts/navigation_transport/navigation_transport.launch`: 
  - Point the `map` argument to your map YAML and adjust `robot_name`/`master_name` if you use TF prefixes.
  - Update the launch file to use automatic_pick.py instead of automatic_pick.
  
- `src/jetauto_example/scripts/navigation_transport/navigation_transport.py`:
  - Use your custom autonomous navigation script instead of RViz services.

- `src/jetauto_example/scripts/navigation_transport/automatic_pick.py`:
  - Motion tuning: `linear_base_speed`, `angular_base_speed`, PID gains, stop pixel targets (`stop_x/stop_y`), and timeouts (`FORCE_PICK_TIMEOUT`, `FORCE_PICK_LOST_TIMEOUT`).
  - Color detection: LAB thresholds are read from `/home/jetauto/jetauto_software/lab_tool/lab_config.yaml`; edit that file to change color ranges.

- `src/jetauto_navigation/launch/include/navigaton_base.launch`:
  - Use map for the local costmap frame to avoid odom drift skewing the footprint.

- RViz: `src/jetauto_example/rviz/navigation_transport.rviz` 
  - Update if you want different panels or topics. Ensure the topics match your script.
  
Note: You do not need to rebuild the workspace.

## Run the demo
From the workspace root after sourcing:
```bash
sudo systemctl stop start_app_node.service.launch
roslaunch jetauto_example navigation_transport.launch
```

Then to start navigating, open a new terminal and run: 
```bash
rosrun jetauto_example navigation_transport.py
```

Note: The robot has trouble localising after pickup sometimes. Future work should focus on this.