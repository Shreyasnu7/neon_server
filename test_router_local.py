
import asyncio
import sys
import os

# Mock Dependencies
sys.path.append(os.getcwd())

from cloud_ai.orchestrator.orchestrator import CloudOrchestrator
from api_schemas import DronePlan

async def test_pipeline():
    print("--- STARTING LOCAL PIPELINE TEST ---")
    
    # 1. Init Orchestrator
    try:
        orch = CloudOrchestrator(drone_id="test_drone_1")
        print("[OK] Orchestrator Initialized")
    except Exception as e:
        print(f"[FAIL] Orchestrator Init Failed: {e}")
        import traceback
        traceback.print_exc()
        return

    # 2. Mock Payload (User "Test Orbit")
    text = "test orbit"
    print(f"INPUT: '{text}'")

    # 3. Process
    try:
        plan_result = await orch.process_multimodal_request(
            text=text,
            user_id="user_1",
            drone_id="test_drone_1",
            api_keys={"gemini": "dummy_key"} 
        )
        print(f"PLAN RESULT RAW: {plan_result}")
    except Exception as e:
        print(f"[FAIL] Process Request Failed: {e}")
        import traceback
        traceback.print_exc()
        return

    # 4. Simulate Router Logic
    final_msg = "UNKNOWN"
    try:
        # Convert to Pydantic
        if isinstance(plan_result, dict):
            # Simulate what Pydantic does
            plan = DronePlan(**plan_result)
            
            # Simulate the recovery logic I added
            if not plan.reasoning and "reasoning" in plan_result:
                plan.reasoning = plan_result["reasoning"]
            
            final_msg = plan.reasoning or "Command queued."
        else:
             final_msg = plan_result.reasoning or "Command queued."
             
        print(f"ROUTER OUTPUT MESSAGE: '{final_msg}'")
        
        if final_msg == "Command queued.":
             print("[FAIL] Still returning default message!")
        else:
             print("[SUCCESS] Returning Logic Message!")

    except Exception as e:
        print(f"[FAIL] Router Logic Failed: {e}")

if __name__ == "__main__":
    asyncio.run(test_pipeline())
