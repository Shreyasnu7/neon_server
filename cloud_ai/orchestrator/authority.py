
class AuthorityManager:
    """
    Manages permissions and command validation for the drone session.
    """
    def __init__(self):
        self.permissions = {
            "admin": ["*"],
            "user": ["flight_control", "camera_control"]
        }
        
    def validate_command(self, user_id: str, command: str) -> bool:
        # Simple permissive check for now
        return True
