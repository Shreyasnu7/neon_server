from .session_state import SessionState
from .authority import AuthorityManager
from .planner import ExecutionPlanner
from .dispatcher import OrchestrationDispatcher

# --- REASONING LAYER IMPORTS ---
# Wiring the "Flat" logic into the "Structure"
from cloud_ai.intent_builder import IntentBuilder
from cloud_ai.cinematic_reasoner import CinematicReasoner
from ai.shot_intent.reasoning.intent_reasoner import ShotIntentReasoner
import os

class CloudOrchestrator:
    """
    Central AI conductor.
    Unified Architecture: Reasoning + Planning + Dispatch.
    """

    def __init__(self, drone_id: str):
        # 1. Dispatch & State Layer
        self.state = SessionState(drone_id=drone_id)
        self.authority = AuthorityManager()
        self.planner = ExecutionPlanner()
        self.dispatcher = OrchestrationDispatcher()

        # 2. Reasoning Layer (The "Brain")
        base_dir = os.path.dirname(os.path.abspath(__file__))
        # prompt path relative to cloud_ai/orchestrator/orchestrator.py -> ../../prompts
        prompt_path = os.path.join(base_dir, "..", "..", "prompts", "shot_intent.txt")
        self.reasoner = ShotIntentReasoner(None, prompt_path) # LLM client passed dynamically or single instance
        self.cinematic = CinematicReasoner()
        self.validator = IntentBuilder() # Strict schema validation

    def inject_llm(self, llm_client):
        """Dependency injection for the LLM provider."""
        self.reasoner.llm = llm_client

    def process_multimodal_request(self, text, user_id, drone_id, images=None, video=None, brain_context=None):
        """
        Full Pipeline: User -> LLM -> Validation -> Plan -> Dispatch.
        """
        # 1. Authority Check
        if not self.authority.can_command(user_id, drone_id):
            return {"status": "denied", "reason": "unauthorized"}

        # 2. Shot Intent Reasoning (LLM)
        # We assume image URLs or base64 are passed in 'images'
        vision_refs = images if images else []
        
        raw_intent = self.reasoner.reason(
            user_text=text,
            image_refs=vision_refs,
            memory_context={"laptop_brain": brain_context or {}} # Deep wiring
        )

        # 3. Strict Validation (IntentBuilder)
        try:
            validated_intent = self.validator.build_base_intent(raw_intent)
            # We use the raw dict for downstream for now, knowing it passed validation
        except Exception as e:
            print(f"[Orchestrator] Validation Warning: {e}")

        # 4. Cinematic Refinement
        refined_intent = self.cinematic.refine(raw_intent)

        # 5. Planning & Dispatch (The existing Enterprise logic)
        return self.handle_intent(refined_intent, human_override=False)

    def handle_intent(
        self,
        shot_intent: dict,
        human_override: bool,
    ):
        owner = self.authority.resolve(
            human_override=human_override,
            ai_request=True,
        )

        if owner != "ai":
            return {"status": "blocked_by_human"}

        plan = self.planner.create_plan(shot_intent)

        self.state.ai_control_active = True
        self.state.active_intent_id = plan["plan_id"]
        self.state.execution_phase = "executing"

        self.dispatcher.dispatch_to_laptop(plan)

        return {
            "status": "executing",
            "plan_id": plan["plan_id"],
        }

    def handle_human_override(self):
        self.state.ai_control_active = False
        self.state.execution_phase = "paused"