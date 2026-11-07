# ByteTrack source and integration

Upstream: https://github.com/FoundationVision/ByteTrack

Pinned commit: `d1bf0191adff59bc8fcfeaa0b33d3d1642552a99`.

The five `.cpp` files in `src/bytetrack` and five `.hpp` files in
`include/k1muse_perception/bytetrack` originate from `deploy/ncnn/cpp/src` and
`deploy/ncnn/cpp/include` at that commit. Upstream `.h` names become `.hpp`.
The MIT notice is retained in `LICENSES/ByteTrack-MIT.txt`; upstream LAPJV code
is included as published in that pinned repository.

Algorithm retained: fixed-step XYAH Kalman filter, IoU/LAPJV global assignment,
high-score association including Lost tracks, low-score continuation of unmatched
Tracked tracks, unconfirmed-track confirmation, reactivation with the original
ID, duplicate removal and the upstream match-cost thresholds for each stage.
No ReID, replacement greedy matcher, detector or inference runtime is imported.

Integration changes, separate from the algorithm:

- Put symbols in `k1muse_perception::bytetrack`; use only OpenCV core headers.
- Constructor accepts score thresholds, an exact Lost-frame buffer and initial
  frame index. No hidden 30-FPS rescaling; wrapper clock stays aligned across labels.
- Carry the matched source detection index through STrack update/reactivation;
  business output copies that actual detection, never the filtered/predicted box.
- The ROS adapter partitions labels, preserves source headers and source order,
  drops duplicate/late source images without advancing state, and resets on
  source geometry/frame changes and normal YOLO session shutdown.
- Bound total Tracked+Lost records by C08 `max_detections`; evict oldest Lost
  records only after matching. Keep only the previous Removed batch, which is
  enough for upstream's delayed Lost subtraction, instead of unbounded history.
- LAPJV uses correctly sized contiguous RAII buffers, replacing upstream arrays
  whose element counts were multiplied by `sizeof` twice. Values/costs are unchanged.
  Invalid-assignment branches throw rather than invoking `system("pause")`/`exit`.
- Remove the upstream startup stdout message and an unused assignment-cost local.

The public adapter is `byte_tracker.hpp`. Tracking is enabled by default.
Task-manager already locks the first eligible ID, considering a replacement only
after more than two seconds of loss and the existing stop/observation conditions.
This policy is separate from ByteTrack's Lost-frame buffer; there is no ten-frame
task failure limit.
The 100ms computation budget must be measured on K1, not inferred from upstream
V100/MOT benchmarks or host tests.
