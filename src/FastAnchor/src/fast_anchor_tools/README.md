# fast_anchor_tools

Utility package for map processing and evaluation tasks around FastAnchor.

Current scripts are buildable placeholders:

- `downsample_pcd.py`
- `crop_pcd.py`
- `transform_pcd.py`
- `evaluate_trajectory.py`
- `plot_icp_score.py`

Install location:

```bash
ros2 run fast_anchor_tools downsample_pcd.py input.pcd output.pcd --leaf-size 0.2
```

These tools are intentionally outside `fast_anchor_localization` and do not
publish the main localization TF tree.
