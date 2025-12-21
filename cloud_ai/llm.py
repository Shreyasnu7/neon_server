
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

    def chat(self, system: str, user: str, provider: str = "gemini") -> str:
        """
        Unified chat interface.
        Returns raw JSON string response.
        """
        full_prompt = f"{system}\n\nUSER REQUEST:\n{user}\n\nOutput JSON only."

        # 1. Try Requested Provider First
        if provider == "gemini" and self.gemini_configured:
             return self._call_gemini(full_prompt)
        elif provider == "openai" and self.openai_client:
             return self._call_openai(system, user)
        
        # 2. Fallback (If requested is missing, try the other)
        if self.gemini_configured:
             return self._call_gemini(full_prompt)
        if self.openai_client:
             return self._call_openai(system, user)

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

    def _call_openai(self, system: str, user: str) -> str:
        try:
            response = self.openai_client.chat.completions.create(
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
            "reasoning": "Fallback: specific cloud keys missing."
        })

# Legacy function alias if something still imports it
def ask_ai(prompt, context=None):
    client = RealLLMClient()
    # simple adapter
    payload = {"prompt": prompt, "context": context or {}}
    return json.loads(client.chat("You are a helper.", json.dumps(payload)))
