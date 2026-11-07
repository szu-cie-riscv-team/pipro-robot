# C10 depth algorithm attribution

`src/depth_yolo_weighted.cpp` is a C++ port of the spatial weights and
depth-centre statistics in Miguel Ángel González Santamarta's `yolo_ros`:

- Source: https://github.com/mgonzs13/yolo_ros/blob/eb11e81f7dbdb81c23a61d743ce0bac3cf9b07eb/yolo_ros/yolo_ros/detect_3d_node.py
- Revision: `eb11e81f7dbdb81c23a61d743ce0bac3cf9b07eb`.
- Copyright (C) 2023 Miguel Ángel González Santamarta.
- License: GPL-3.0-or-later; text in `LICENSES/GPL-3.0.txt`.

The port adds shared image/range validation and a representative pixel from
the contributing depth samples. It omits masks, object extents/orientation,
keypoints, TF, and the Python node. Equal-depth sample ties use pixel order;
floating-point operation order may differ from NumPy. Depth fixture values
are generated from the pinned upstream statistics for regression comparison.

Original project components use the repository's Apache-2.0 license, reproduced
in `LICENSES/Apache-2.0.txt`. The combined C10 support library and executable
include the GPL port and are distributed under GPL-3.0-or-later, with both
original and upstream notices retained. The Apache declaration does not
relicense the upstream port.
