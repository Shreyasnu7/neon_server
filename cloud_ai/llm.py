
import os
import json
import logging

try:
    import google.generativeai as genai
except ImportError:
    genai = None

try:
    from openai import OpenAI
except ImportError:
    OpenAI = None

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
        # DEBUG: Verify Keys
        print(f"DEBUGGING LLM: Provider={provider}")
        print(f"DEBUGGING LLM: Keys Present: {list(api_keys.keys())}")
        if provider == "gemini":
             print(f"DEBUGGING LLM: Gemini Key Length: {len(api_keys.get('gemini', '') or os.getenv('GEMINI_API_KEY', '') or '')}")
        
        full_prompt = f"{system}\n\nUSER REQUEST:\n{user}\n\nOutput JSON only."
        
        # Dynamic Configuration from Client Keys
        client_gemini_key = api_keys.get("gemini")
        client_openai_key = api_keys.get("openai")
        
        # 1. Try Requested Provider First
        if provider == "gemini":
             if client_gemini_key and genai:
                 try:
                    genai.configure(api_key=client_gemini_key)
                    return self._call_gemini(full_prompt)
                 except Exception as e:
                    logger.error(f"Client Gemini Key Failed: {e}")
             
             if self.gemini_configured:
                 return self._call_gemini(full_prompt)

        elif provider == "openai":
             if client_openai_key and OpenAI:
                 try:
                    temp_client = OpenAI(api_key=client_openai_key)
                    return self._call_openai(system, user, client=temp_client)
                 except Exception as e:
                    logger.error(f"Client OpenAI Key Failed: {e}")

             if self.openai_client:
                 return self._call_openai(system, user)
        
        # 2. Fallback (If requested is missing, try the other)
        # Check Env Vars first
        if self.gemini_configured: return self._call_gemini(full_prompt)
        if self.openai_client: return self._call_openai(system, user)
        
        # Check Client Keys second
        if client_gemini_key and genai:
            try:
                genai.configure(api_key=client_gemini_key)
                return self._call_gemini(full_prompt)
            except: pass
            
        if client_openai_key and OpenAI:
             try:
                temp_client = OpenAI(api_key=client_openai_key)
                return self._call_openai(system, user, client=temp_client)
             except: pass

        # 3. Fail
        logger.warning("⚠️ Using Fallback Mock (No Real AI Available)")
        return self._mock_response(user)

    def _call_gemini(self, prompt: str) -> str:
        if not genai:
             logger.error("Gemini module missing")
             return None
        try:
            model = genai.GenerativeModel('gemini-1.5-flash')
            response = model.generate_content(prompt)
            return self._clean_json(response.text)
        except Exception as e:
            logger.error(f"Gemini Error: {e}")
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
            logger.error(f"OpenAI Error: {e}")
            return None

    def _clean_json(self, text: str) -> str:
        return text.replace("```json", "").replace("```", "").strip()

    def _mock_response(self, user_input: str) -> str:
        """Simple fallback if keys are missing"""
        # Try to parse user input to give something semi-relevant if possible, 
        # but mostly return a safe default.
        return json.dumps({
            "emotional_model": {"vector": {"neutral": 1.0}, "peak_allowed": True}, 
            "camera_plan": {"shot_energy": 0.5, "framing": "wide"},
            "reasoning": "Fallback: AI Service Unavailable (Check Keys)"
        })

# Legacy function alias if something still imports it
def ask_ai(prompt, context=None):
    client = RealLLMClient()
    # simple adapter
    payload = {"prompt": prompt, "context": context or {}}
    return json.loads(client.chat("You are a helper.", json.dumps(payload)))
