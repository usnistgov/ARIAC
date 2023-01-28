# TF Frames

This section describes the TF frames used in the competition. The TF frames are used to describe the position and orientation of the different objects in the competition. After starting the environment, one can visualize the TF tree with the following command:

```bash
cd /tmp && ros2 run tf2_tools view_frames  && evince frames.pdf
```

![drawing](../images/AdvancedLogicalCamera.png)



A PDF file containing the TF frames is also available [here](../images/frames.pdf). A summary of the TF frames used in the competition is provided below:

* **World**: The `world` frame is the root frame of the TF tree. It is located at the origin of the competition arena.
* **AGV Trays**: `agv1_tray`, `agv2_tray`, `agv3_tray`, and `agv4_tray` frames are located at the origin of the AGV trays.

    ```
    world
    └─── agv1_track
    |    |
    |    └─── agv1_base
    |            |
    |            └─── agv1_tray
    └─── agv2_track
        └─── agv2_base
                └─── agv2_tray
    └─── agv3_track
        └─── agv3_base
                └─── agv3_tray
    └─── agv4_track
        └─── agv4_base
                └─── agv1_tray
    ```
