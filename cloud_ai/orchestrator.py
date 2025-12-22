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
        user_text = payload.get("text", "")
        references = payload.get("media", []) or [] 
        api_keys = payload.get("api_keys", {})

        # 1. RAW REASONING (The "Director" Brain)
        if self.intent_reasoner:
            # Detect if there is real vision data (detections) in the references
            # We assume 'image_refs' might hold YOLO JSON objects in this pipeline version
            # or we get it from 'memory_context'. Safely filter to avoid crashing on base64 strings.
            
            vision_context = [r for r in references if isinstance(r, dict) and r.get('type') == 'vision_detection']
            
            try:
                raw_intent = self.intent_reasoner.reason(
                    user_text=user_text,
                    image_refs=vision_context,
                    memory_context={"history": []}, 
                    provider=payload.get("provider", "gemini"),
                    api_keys=api_keys
                )
            except Exception as e:
                print(f"Orchestrator Error: {e}")
                # Fallback Intent
                raw_intent = {"emotional_model": {"vector": {"neutral": 1.0}, "peak_allowed": True}, "camera_plan": {"shot_energy": 0.5}}
        else:
            # Fallback if no LLM client passed (or mock)
            raw_intent = {"emotional_model": {"vector": {"neutral": 1.0}, "peak_allowed": True}, "camera_plan": {"shot_energy": 0.5}}

        # 2. CINEMATIC REFINEMENT (The "Cinematographer" Brain)
        refined_intent = self.cinematic_reasoner.refine(raw_intent)

        # 3. PLAN GENERATION (The "Assistant Director" Brain)
        final_plan = self.planner.generate(refined_intent)

        return final_plan
