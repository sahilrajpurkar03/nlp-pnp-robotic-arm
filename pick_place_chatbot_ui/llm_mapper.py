# llm_mapper.py
import re

VALID_LABELS = [
    "banana", "marker", "rubiks_cube",
    "green_cube", "red_cube", "yellow_cube",
    "scissor", "clamp"
]

def llm_chat_or_pick(user_text: str) -> dict:
    # Check if user wants to pick something
    for label in VALID_LABELS:
        if re.search(rf"\b{label}\b", user_text, re.IGNORECASE):
            return {
                "type": "pick",
                "label": label,
                "reply": f"Picking the {label} 🟢"
            }

    # If no valid pick detected, fallback to LLM for chat
    prompt = f"""
You are a helpful robot assistant. Only reply to casual chat.
User message: "{user_text}"
Respond only with text (no JSON needed).
"""
    import subprocess, json
    result = subprocess.run(
        ["ollama", "run", "phi3", prompt],
        capture_output=True, text=True
    )
    reply = result.stdout.strip() or "Hello! How can I help you?"
    
    return {
        "type": "chat",
        "reply": reply
    }
