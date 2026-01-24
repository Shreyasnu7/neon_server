from cloud_ai.orchestrator import CloudOrchestrator as Orchestrator

_orchestrator = Orchestrator()

def get_orchestrator():
    return _orchestrator
