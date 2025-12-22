from .intent_builder import IntentBuilder
from .cinematic_reasoner import CinematicReasoner
from .plan_generator import PlanGenerator
from ai.shot_intent.reasoning.intent_reasoner import ShotIntentReasoner
# Assuming you have a class for multi-modal acting as the entry reasoning
# If not, we will use ShotIntentReasoner as the primary entry

class CloudOrchestrator:
    """
    The Master Conductor.
    Pipeline:
    1. Multi-modal Ingest (Text/Image/Video) -> ShotIntentReasoner
    2. Cinematic Refinement -> CinematicReasoner
    3. Plan Generation -> PlanGenerator
    """
    def __init__(self, llm_client=None):
        # We need to instantiate the sub-modules
        # Note: In a real app, inject dependencies or config
        import os
        base_dir = os.path.dirname(os.path.abspath(__file__))
        prompt_path = os.path.join(base_dir, "..", "prompts", "shot_intent.txt")
        self.intent_reasoner = ShotIntentReasoner(llm_client, prompt_path) if llm_client else None
        self.cinematic_reasoner = CinematicReasoner()
        self.planner = PlanGenerator()

    async def process_request(self, payload: dict) -> dict:
        """
        Full Pipeline Execution
        """
        refined_intent = self.cinematic_reasoner.refine(raw_intent)

        # 3. PLAN GENERATION (The "Assistant Director" Brain)
        # Converts abstract intent into specific Drone Waypoints/Commands
        final_plan = self.planner.generate(refined_intent)

        return final_plan
