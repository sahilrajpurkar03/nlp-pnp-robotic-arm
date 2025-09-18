# llm_mapper.py
import re
import difflib
import subprocess

VALID_LABELS = [
    "banana", "marker", "rubiks_cube",
    "green_cube", "red_cube", "yellow_cube",
    "scissor", "clamp", "allen_key", "measuring_tape",
    "knife", "chisel"
]

PICK_SYNONYMS = ["pick", "take", "grab", "collect", "lift", "get"]

LABEL_SYNONYMS = {
    "red_cube": ["red block", "red box"],
    "green_cube": ["green block", "green box"],
    "yellow_cube": ["yellow block", "yellow box"],
    "rubiks_cube": ["rubik's cube", "rubix cube"],
    "measuring_tape": ["tape", "measuring tape roll", "tape measure"],
    "clamp": ["clamp tool", "clamp device"],
    # add more as needed
}

# keep track of last suggestion for yes/no confirmation
_last_suggestion = None

def find_best_label(user_text: str) -> str | None:
    words = user_text.lower().split()
    best_match = None
    best_ratio = 0.0

    for label in VALID_LABELS:
        candidates = [label] + label.split("_")  # original parts
        # include synonyms if defined
        if label in LABEL_SYNONYMS:
            candidates += LABEL_SYNONYMS[label]

        for word in words:
            for cand in candidates:
                ratio = difflib.SequenceMatcher(None, word, cand.lower()).ratio()
                if ratio > best_ratio:
                    best_match = label
                    best_ratio = ratio

    return best_match if best_ratio > 0.6 else None



def llm_chat_or_pick(user_text: str) -> dict:
    """
    Determines if user wants to pick an object (exact or fuzzy match),
    asks for confirmation if needed, or falls back to casual chat.
    """
    global _last_suggestion
    text = user_text.lower().strip()

    # --- Handle confirmation responses ---
    if _last_suggestion:
        if text in ["yes", "y", "yeah", "ok", "sure"]:
            label = _last_suggestion
            _last_suggestion = None
            return {
                "type": "pick",
                "label": label,
                "reply": f"Okay, picking the {label.replace('_',' ')} 🟢"
            }
        elif text in ["no", "n", "nope"]:
            _last_suggestion = None
            return {
                "type": "chat",
                "reply": "Alright, cancelled ❌. What should I grab instead?"
            }

    # --- Detect pick intent ---
    if any(re.search(rf"\b{syn}\b", text) for syn in PICK_SYNONYMS):

        # ✅ Exact match (with or without underscore/space)
        for label in VALID_LABELS:
            pattern = label.replace("_", "[ _]")
            if re.search(rf"\b{pattern}\b", text):
                return {
                    "type": "pick",
                    "label": label,
                    "reply": f"Picking the {label.replace('_',' ')} 🟢"
                }

        # ✅ Fuzzy/partial match (typos and partial words)
        best_match = find_best_label(text)
        if best_match:
            _last_suggestion = best_match
            return {
                "type": "chat",
                "reply": f"Did you mean **{best_match.replace('_',' ')}**? (yes/no)"
            }

        # if no match at all
        return {
            "type": "chat",
            "reply": "I didn’t recognize that item 🤔. Can you try again?"
        }

    # --- Fallback: LLM casual chat ---
    prompt = f"""
You are a helpful robot assistant. Only reply to casual chat.
User message: "{user_text}"
Respond only with text (no JSON needed).
"""
    result = subprocess.run(
        ["ollama", "run", "phi3", prompt],
        capture_output=True, text=True
    )
    reply = result.stdout.strip() or "Hello! How can I help you?"

    return {
        "type": "chat",
        "reply": reply
    }
