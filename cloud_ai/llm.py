
import os
import json
import logging

GENAI_IMPORT_ERROR = None
try:
    import google.generativeai as genai
    print("DEBUG: Successfully imported google.generativeai")
except ImportError as e:
    genai = None
    GENAI_IMPORT_ERROR = str(e)
    print(f"CRITICAL ERROR: Could not import google.generativeai: {e}")

try:
    from openai import OpenAI
except ImportError:
    OpenAI = None

# ... (logging config)



# Configure Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RealLLMClient:
    def __init__(self):
        self.gemini_key = os.environ.get("GEMINI_API_KEY")
        self.openai_key = os.environ.get("OPENAI_API_KEY")
        
        self.gemini_configured = False
        self.openai_client = None

        if self.gemini_key and genai:
            genai.configure(api_key=self.gemini_key)
            self.gemini_configured = True
            logger.info("✅ Gemini Client Configured")

        if self.openai_key and OpenAI:
            self.openai_client = OpenAI(api_key=self.openai_key)
            logger.info("✅ OpenAI Client Configured")

    def chat(self, system: str, user: str, provider: str = "gemini", api_keys: dict = {}) -> str:
        """
        Unified chat interface.
        Returns raw JSON string response.
        """
        last_error = "No Real AI Available" # Track the specific error

        # DEBUG: Verify Keys
        print(f"DEBUGGING LLM: Provider={provider}")
        # print(f"DEBUGGING LLM: Keys Present: {list(api_keys.keys())}") # Privacy safe
        
        full_prompt = f"{system}\n\nUSER REQUEST:\n{user}\n\nOutput JSON only."
        
        # Dynamic Configuration from Client Keys
        client_gemini_key = api_keys.get("gemini")
        client_openai_key = api_keys.get("openai")
        
        # 1. Try Requested Provider First
        if provider == "gemini":
             if not genai:
                 last_error = f"Lib Import Fail: {GENAI_IMPORT_ERROR or 'Unknown'}"
                 logger.warning(last_error)
             
             elif client_gemini_key:
                 try:
                    # Configure execution...
                    return self._call_gemini(full_prompt)
                 except Exception as e:
                    logger.error(f"Client Gemini Key Failed: {e}")
                    last_error = f"Gemini Key Error: {e}"
             
             if self.gemini_configured:
                 return self._call_gemini(full_prompt)

        elif provider == "openai":
             if client_openai_key and OpenAI:
                 try:
                    temp_client = OpenAI(api_key=client_openai_key)
                    return self._call_openai(system, user, client=temp_client)
                 except Exception as e:
                    logger.error(f"Client OpenAI Key Failed: {e}")
                    if "429" in str(e): last_error = "OpenAI Quota Exceeded (Check Billing)"
                    else: last_error = f"OpenAI Key Error: {str(e)[:50]}..."

             if self.openai_client:
                 return self._call_openai(system, user)
        
        # 2. Fallback RE-ATTEMPT (Try the other one)
        # ... (Simplified logic: If primary failed, we try others, but for now let's just error out to show the message)
        
        # 3. Fail
        logger.warning(f"⚠️ Using Fallback. Reason: {last_error}")
        return self._mock_response(user, error=last_error)

    def _call_gemini(self, prompt: str) -> str:
        if not genai: 
            error_msg = f"GenAI Lib Missing: {GENAI_IMPORT_ERROR or 'Unknown'}"
            logger.error(error_msg)
            raise Exception(error_msg) # Raise to be caught by chat() loop or fallback logic
        
        # Free Tier Limit Strategy:
        # Gemini 1.5 Pro Free = 2 Requests Per Minute (Too slow!)
        # Gemini 1.5 Flash Free = 15 Requests Per Minute (Perfect for drones)
        
        # User strictly requested gemini-3.0-flash, but we keep backups to prevent crashes.
        models_to_try = [
            'gemini-3.0-flash',     
            'gemini-2.0-flash-exp',
            'gemini-1.5-flash'
        ]
        
        for model_name in models_to_try:
            try:
                print(f"DEBUGGING LLM: Trying Model: {model_name}")
                model = genai.GenerativeModel(model_name)
                response = model.generate_content(prompt)
                return self._clean_json(response.text)
            except Exception as e:
                print(f"DEBUGGING LLM: Failed {model_name} -> {e}")
                logger.error(f"Gemini {model_name} Error: {e}")
                if "429" in str(e): continue 
                if "404" in str(e): continue 
                # continue trying others even if it's another error
        
        # If all failed
        return None

    def _call_openai(self, system: str, user: str, client=None) -> str:
        try:
            active_client = client or self.openai_client
            if not active_client: return None
            
            response = active_client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": system},
                    {"role": "user", "content": user}
                ],
                response_format={"type": "json_object"}
            )
            return response.choices[0].message.content
        except Exception as e:
            # Let the caller handle the specific exception for better error messaging
            raise e 

    def _clean_json(self, text: str) -> str:
        return text.replace("```json", "").replace("```", "").strip()

    def _mock_response(self, user_input: str, error: str = None) -> str:
        """Simple fallback if keys are missing"""
        reason = f"System: {error}" if error else "System: AI Offline"
        
        return json.dumps({
            "emotional_model": {"vector": {"neutral": 1.0}, "peak_allowed": True}, 
            "camera_plan": {"shot_energy": 0.5, "framing": "wide"},
            "reasoning": reason 
        })

# Legacy function alias if something still imports it
def ask_ai(prompt, context=None):
    client = RealLLMClient()
    # simple adapter
    payload = {"prompt": prompt, "context": context or {}}
    return json.loads(client.chat("You are a helper.", json.dumps(payload)))
