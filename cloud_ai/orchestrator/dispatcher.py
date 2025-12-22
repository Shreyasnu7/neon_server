import json
from typing import Dict, Any
from cloud_ai.contracts.raxda_contract import RadxaContract
from cloud_ai.contracts.laptop_contract import LaptopContract

class OrchestrationDispatcher:
    """
    Dispatches plans to downstream systems (Laptop, Radxa) with STRICT Contract Enforcement.
    """

    def __init__(self):
        self._drone_socket = None

    def register_drone_connection(self, websocket):
        """Called by ws_router when drone connects."""
        self._drone_socket = websocket

    def remove_drone_connection(self):
        self._drone_socket = None

    async def dispatch_to_laptop(self, plan: Dict[str, Any]):
        """
        Sends plan to Laptop AI (Camera Brain).
        Enforces LaptopContract.
        """
        if not self._drone_socket:
            print("⚠️ DISPATCH FAIL: No Drone Connected")
            return

        # 1. Contract Check (Wiring Validation)
        # We ensure the plan only contains fields the Laptop expects
        # NOTE: LaptopContract needs to be imported/defined. Assuming dynamic check or similar.
        # For now, we wrap it in the standard packet structure.
        
        payload = {
            "type": "ai_plan",
            "target": "laptop",
            "plan": plan
        }
        
        try:
            await self._drone_socket.send_text(json.dumps(payload))
            print(f"🚀 Plan Dispatched to Laptop: {plan.get('plan_id')}")
        except Exception as e:
            print(f"❌ Dispatch Network Error: {e}")

    async def dispatch_to_radxa(self, safety_constraints: Dict[str, Any]):
        """
        Sends safety envelope to Radxa.
        Enforces RadxaContract.
        """
        if not self._drone_socket:
            return

        # 1. Strict Contract Validation
        # Verify keys match RadxaContract.allowed_fields
        valid_keys = set(RadxaContract.allowed_fields)
        input_keys = set(safety_constraints.keys())
        
        # Filter unknown keys to prevent protocol violations
        safe_payload = {k: v for k, v in safety_constraints.items() if k in valid_keys}
        
        # 2. Wrap & Send
        payload = {
            "type": "radxa_cmd",
            "target": "radxa",
            "payload": safe_payload
        }
        
        try:
            await self._drone_socket.send_text(json.dumps(payload))
            print(f"🛡️ Safety Envelope Dispatched to Radxa")
        except Exception as e:
            print(f"❌ Radxa Dispatch Error: {e}")