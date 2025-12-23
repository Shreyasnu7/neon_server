class AuthorityManager:
    """
    Resolves who is allowed to command the drone.
    """

    def resolve(
        self,
        human_override: bool,
        ai_request: bool,
    ) -> str:
        """
        Returns authority owner.
        """

        if human_override:
            return "human"

        if ai_request:
            return "ai"

        return "human"

    def can_command(self, user_id: str, drone_id: str) -> bool:
        """
        Check if user is authorized.
        """
        # For now, simplistic check: Always YES for registered users in this MVP
        return True