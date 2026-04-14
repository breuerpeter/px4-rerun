# Blueprints

Rerun blueprint files that customize the viewer layout for PX4 data.

| Blueprint | Description |
|---|---|
| `trajectory.rbl` | Single 3D view, excludes setpoint/velocity overlays |
| `vehicle.rbl` | Single 3D view, orbital camera tracking the vehicle body |
| `vehicle_left_trajectory_right.rbl` | Side-by-side vehicle + trajectory views |
| `messages.rbl` | Side-by-side PX4 text log + trajectory views |

## Regenerating

Note: requires [uv](https://docs.astral.sh/uv/)

```bash
uv run --with "rerun-sdk==0.31.1" python blueprints/generate.py
```
