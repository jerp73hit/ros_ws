#!/usr/bin/env python3
import os
import sys
import subprocess
import importlib
import rospy
from ollama_client import OllamaChatClient
from llm_api import init_api

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.realpath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
GENERATED_PATH = os.path.join(SCRIPT_DIR, "generated_task.py")

scan_script = os.path.join(SCRIPT_DIR, "go_to_top_hand_camera_pos.py")
print("[run] Moving to scan pose...", flush=True)
subprocess.run([sys.executable, scan_script], check=True)
print("[run] Scan pose reached.", flush=True)

sys_prompt_path = os.path.join(WORKSPACE, "system_prompt.md")
if not os.path.exists(sys_prompt_path):
    print(f"[run] system_prompt.md not found at {sys_prompt_path}")
    sys.exit(1)
with open(sys_prompt_path) as f:
    SYSTEM_PROMPT = f.read()

client = OllamaChatClient(system_prompt=SYSTEM_PROMPT)

if len(sys.argv) > 1:
    user_text = " ".join(sys.argv[1:])
    print(f"[run] Task: {user_text}")
else:
    user_text = input("Describe the task: ")

result = client.generate_response(user_text)
if result is None:
    print("[run] LLM returned no response")
    sys.exit(1)
result = result.strip()

if result.startswith("```"):
    lines = result.splitlines()
    if lines and lines[0].startswith("```"):
        lines.pop(0)
    if lines and lines[-1].strip() == "```":
        lines.pop(-1)
    result = "\n".join(lines).strip()

if not os.path.exists(GENERATED_PATH):
    with open(GENERATED_PATH, "w") as f:
        f.write("def execute_task():\n    pass\n")

with open(GENERATED_PATH, "w") as f:
    f.write("from llm_api import make_scene, adjust_grasp_waypoints, execute_waypoints\n\n")
    f.write(result)
    if not result.endswith("\n"):
        f.write("\n")

print(f"[run] Wrote task to {GENERATED_PATH}")

if "generated_task" in sys.modules:
    importlib.reload(sys.modules["generated_task"])
else:
    import generated_task

if not hasattr(generated_task, "execute_task"):
    print("[run] generated_task.py does not contain execute_task()")
    sys.exit(1)

init_api()
rospy.sleep(1.0)

print("[run] Executing task...")
try:
    generated_task.execute_task()
    print("[run] ✓ Task completed successfully.")
except Exception as e:
    print(f"[run] ✗ Task failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
