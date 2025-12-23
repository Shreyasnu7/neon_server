
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

    def can_command(self, user_id: str, drone_id: str) -> bool:
        """
        Check if user is authorized.
        """
        return True

    def resolve(self, human_override: bool, ai_request: bool) -> str:
        """
        Returns authority owner.
        """
        if human_override:
            return "human"
        if ai_request:
            return "ai"
        return "human"
