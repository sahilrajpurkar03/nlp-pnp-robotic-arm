# llm_mapper.py
import subprocess

VALID_LABELS = [
    "banana", "marker", "rubiks_cube",
    "green_cube", "red_cube", "yellow_cube",
    "scissor", "clamp"
]

def llm_extract_label(user_text: str) -> str | None:
    prompt = f"""
    You are a command interpreter for a robot arm.
    The user will ask to pick an object.
    Valid objects are: {", ".join(VALID_LABELS)}.
    From the user request below, extract the correct object label.
    Respond with only the exact label from the list, nothing else.

    Request: {user_text}
    """

    # Call Ollama with a small model (phi3 or qwen2.5)
    result = subprocess.run(
        ["ollama", "run", "phi3", prompt],
        capture_output=True, text=True
    )

    output = result.stdout.strip()
    if output in VALID_LABELS:
        return output
    return None
