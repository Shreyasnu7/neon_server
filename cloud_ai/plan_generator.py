# server/cloud_ai/plan_generator.py

from api_schemas import DronePlan


class PlanGenerator:
    """
    Converts AI reasoning output into a DronePlan.
    """

    def generate(self, ai_result: dict) -> DronePlan:
        # Extract Nested Plans
        cam = ai_result.get("camera_plan", {})
        gopro = cam.get("gopro_settings", {})
        gimbal = cam.get("gimbal", {})

        return DronePlan(
            action=ai_result.get("intent_type", "hover"), # Mapped intent_type -> action
            style="cinematic",
            target=ai_result.get("subject"),
            constraints={},
            reasoning=ai_result.get("reasoning", "Executing cinematic command."),
            
            # Detailed Hollywood Config
            flight_behavior=cam.get("movement_style", "standard"),
            camera_config={
                "resolution": gopro.get("resolution", "5.3K"),
                "framerate": gopro.get("fps", 60),
                "shutter": gopro.get("shutter", "auto"),
                "iso_max": gopro.get("iso_limit", 1600),
                "color_profile": gopro.get("color", "flat"),
                "lens": gopro.get("lens", "linear")
            },
            gimbal_config={
                "mode": gimbal.get("mode", "follow"),
                "pitch_angle": float(gimbal.get("pitch", 0.0))
            },
            
            # Global Tone Curve Integration
            post_process={
                "lut_style": cam.get("post_processing", {}).get("lut_style", "ACES_Standard"),
                "global_tone_curve": {
                    "contrast_stretch": 1.0,
                    "highlight_rolloff": cam.get("post_processing", {}).get("tone_curve_params", {}).get("highlight_rolloff", 0.8),
                    "shadow_toe": cam.get("post_processing", {}).get("tone_curve_params", {}).get("shadow_toe", 0.15)
                },
                "super_resolution": cam.get("post_processing", {}).get("super_resolution", False)
            }
        )