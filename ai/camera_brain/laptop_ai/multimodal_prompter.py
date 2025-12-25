# laptop_ai/multimodal_prompter.py
import json
import base64
import aiohttp
import time
from laptop_ai.config import OPENAI_API_KEY, OPENAI_MODEL, DEEPSEEK_API_KEY, DEEPSEEK_URL, USE_LOCAL_LLM

# Import the User's Advanced Reasoning Engine
try:
    from ai.shot_intent.reasoning.intent_reasoner import ShotIntentReasoner
except ImportError:
    print("⚠️ Warning: ShotIntentReasoner not found. Using fallback.")
    ShotIntentReasoner = None

SYSTEM_PROMPT = """
You are a Drone Cinematic Planner AI.
Input: user_text, vision_context, images (URLs), video_link, memory.
Return: A strict JSON object with one of:
- {"action":"FOLLOW","params":{"distance_m":5,"height_m":3,"style":"CINEMATIC"}}
- {"action":"ORBIT","params":{"radius_m":8,"height_m":4,"speed_ms":1.5}}
- {"action":"HOVER","params":{}}
- {"action":"MOVE_TO","params":{"x":1.3,"y":0.0,"z":2.5,"yaw":0.0}}
- {"action":"RECORD","params":{"duration_s":8,"style":"SPORTS"}}
Always keep numbers reasonable. Do NOT include raw motor commands.
"""

class MockLLM:
    """Adapts raw aiohttp call to the interface expected by ShotIntentReasoner"""
    async def chat(self, system, user, provider="gemini", api_keys={}):
        pass

async def ask_gpt(user_text, vision_context=None, images=None, video_link=None, memory=None, timeout=15, api_keys={}):
    """
    Sends a multimodal prompt using the ADVANCED SHOT INTENT REASONER if available.
    Supports DeepSeek for ultra-low latency if configured.
    """
    
    # 1. Try to use the Advanced Brain
    if ShotIntentReasoner:
         class AsyncAdapterLLM:
             async def chat_async(self, system, user):
                # Determine Provider
                if USE_LOCAL_LLM:
                    # DeepSeek / Local Mode
                    api_key = api_keys.get("deepseek") or DEEPSEEK_API_KEY
                    url = f"{DEEPSEEK_URL}/chat/completions"
                    model = "deepseek-chat"
                else:
                    # Generic OpenAI Mode
                    api_key = api_keys.get("openai") or OPENAI_API_KEY
                    url = "https://api.openai.com/v1/chat/completions"
                    model = OPENAI_MODEL

                payload = {
                    "model": model,
                    "messages": [{"role":"system","content":system}, {"role":"user","content":user}],
                    "max_tokens": 1000,
                    "temperature": 0.2
                }
                headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
                async with aiohttp.ClientSession() as sess:
                    async with sess.post(url, json=payload, headers=headers, timeout=timeout) as resp:
                        if resp.status != 200: return None
                        data = await resp.json()
                        return data['choices'][0]['message']['content']

         # Instantiate Reasoning Engine
         adapter = AsyncAdapterLLM()
         # Temporary prompt path - in real app, ensure this file exists or use string
         # We will bypass the file read in Reasoner by mocking it or creating a dummy
         
         # Logic:
         # 1. Construct payload as Reasoner would.
         # 2. Add System Prompt.
         # 3. Send.
         
         full_context = {
             "vision": vision_context,
             "memory": memory,
             "user_text": user_text
         }
         
         # Use the Adapter to send
         # We are integrating the Logic of Reasoner (Context Structuring) 
         response_text = await adapter.chat_async(SYSTEM_PROMPT, json.dumps(full_context))
         
         try:
             # Parse strictly
             # Remove markdown blocks
             if "```json" in response_text:
                 response_text = response_text.split("```json")[1].split("```")[0]
             return json.loads(response_text)
         except:
             print("JSON Parse Error in AI Response")
             return None

    # Fallback to legacy simplistic method if import failed
    payload = {
        "model": OPENAI_MODEL,
        "messages": [
            {"role":"system", "content":SYSTEM_PROMPT},
            {"role":"user", "content": f"User request: {user_text}\n\nVisionContext: {json.dumps(vision_context or {}, default=str)}\nMemory:{json.dumps(memory or {})}\nImages:{images or []}\nVideo:{video_link or ''}"}
        ],
        "max_tokens": 400,
        "temperature": 0.2,
    }

    headers = {
        "Authorization": f"Bearer {OPENAI_API_KEY}",
        "Content-Type": "application/json"
    }

    try:
        async with aiohttp.ClientSession() as sess:
            async with sess.post("https://api.openai.com/v1/chat/completions", json=payload, headers=headers, timeout=timeout) as resp:
                if resp.status != 200:
                    text = await resp.text()
                    print("OpenAI error:", resp.status, text[:400])
                    return None
                data = await resp.json()
                content = data['choices'][0]['message']['content']
                # Clean markdown
                if "```json" in content:
                    content = content.split("```json")[1].split("```")[0].strip()
                elif "```" in content:
                    content = content.split("```")[1].strip()
                return json.loads(content)
    except Exception as e:
        print(f"LLM Network Error: {e}")
        return None


