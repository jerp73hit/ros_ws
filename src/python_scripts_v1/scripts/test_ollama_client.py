#!/usr/bin/env python3
import sys
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.realpath(os.path.join(_SCRIPT_DIR, "..", "..", ".."))

sys.path.insert(0, _SCRIPT_DIR)

from ollama_client import OllamaChatClient


def load_system_prompt(path=None):
    if path is None:
        path = os.path.join(_WS_ROOT, "system_prompt.md")
    with open(path, "r") as f:
        return f.read()


def main():
    system_prompt = load_system_prompt()

    client = OllamaChatClient(system_prompt=system_prompt, model="qwen2.5-coder:3b")

    if len(sys.argv) > 1:
        user_input = " ".join(sys.argv[1:])
    else:
        user_input = input("Enter your instruction: ")

    result = client.generate_response(user_input)

    if result:
        print(result)
    else:
        print("Failed to get a response from Ollama.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
