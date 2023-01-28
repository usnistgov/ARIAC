# TF Frames

This section describes the TF frames used in the competition. The TF frames are used to describe the position and orientation of the different objects in the competition. After starting the environment, one can visualize the TF tree with the following command:

```bash
cd /tmp && ros2 run tf2_tools view_frames  && evince frames.pdf
```

A PDF file containing the TF frames is also available [here](../images/frames.pdf).

A summary of the TF frames used in the competition is provided below:
    * **world**: The world frame is the root frame of the TF tree. It is located at the origin of the competition arena.

project
│   README.md
│   file001.txt
│
└───folder1
│   │   file011.txt
│   │   file012.txt
│   │
│   └───subfolder1
│       │   file111.txt
│       │   file112.txt
│       │   ...
│
└───folder2
    │   file021.txt
    │   file022.txt
