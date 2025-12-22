# cloud_ai/dependencies.py
from cloud_ai.orchestrator.orchestrator import CloudOrchestrator
from cloud_ai.llm import RealLLMClient

# SINGLETON INSTANCE
# This holds the state for all connected drones.
# In a real enterprise app, this would be a cache/database, but for this single-server instance, memory is fine.
global_orchestrator = CloudOrchestrator(drone_id="default_drone")

# Inject Dependencies
_llm = RealLLMClient()
global_orchestrator.inject_llm(_llm)

def get_orchestrator() -> CloudOrchestrator:
    return global_orchestrator
