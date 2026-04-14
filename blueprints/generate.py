#!/usr/bin/env python3
"""Generate Rerun blueprint (.rbl) files for the viewer."""

from pathlib import Path

import rerun.blueprint as rrb

APP_ID = "px4-rerun"
OUTPUT_DIR = Path(__file__).parent

COLLAPSED_PANELS = (
    rrb.BlueprintPanel(state="collapsed"),
    rrb.SelectionPanel(state="collapsed"),
    rrb.TimePanel(state="collapsed"),
)

blueprints = {
    "trajectory": rrb.Blueprint(
        rrb.Spatial3DView(
            origin="/",
            contents=[
                "+ $origin/**",
                "- /px4/world/setpoint/**",
                "- /px4/world/setpoint_yaw/**",
                "- /px4/world/setpoint_yaw_label/**",
                "- /px4/world/velocity/**",
            ],
        ),
        *COLLAPSED_PANELS,
    ),
    "vehicle": rrb.Blueprint(
        rrb.Spatial3DView(
            origin="/",
            eye_controls=rrb.archetypes.EyeControls3D(
                kind="Orbital",
                tracking_entity="px4/body/frd",
            ),
        ),
        *COLLAPSED_PANELS,
    ),
    "both": rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="/",
                eye_controls=rrb.archetypes.EyeControls3D(
                    kind="Orbital",
                    tracking_entity="px4/body/frd",
                ),
            ),
            rrb.Spatial3DView(
                origin="/",
                contents=[
                    "+ $origin/**",
                    "- /px4/world/setpoint/**",
                    "- /px4/world/setpoint_yaw/**",
                    "- /px4/world/setpoint_yaw_label/**",
                    "- /px4/world/velocity/**",
                ],
            ),
        ),
        *COLLAPSED_PANELS,
    ),
}

for name, blueprint in blueprints.items():
    path = OUTPUT_DIR / f"{name}.rbl"
    blueprint.save(APP_ID, str(path))
    print(f"  {path}")

print(f"Generated {len(blueprints)} blueprint(s)")
