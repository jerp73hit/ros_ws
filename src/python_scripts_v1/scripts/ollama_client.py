import requests


class OllamaChatClient:
    def __init__(self, api_key="", system_prompt="", model="qwen2.5-coder:3b", api_url=None):
        self.api_key = api_key
        self.system_prompt = system_prompt
        self.model = model
        self.api_url = api_url or "http://localhost:11434/v1/chat/completions"

        self.headers = {
            "Content-Type": "application/json",
        }
        if self.api_key:
            self.headers["Authorization"] = f"Bearer {self.api_key}"

    def update_system_prompt(self, new_prompt):
        self.system_prompt = new_prompt

    def generate_response(self, user_input, temperature=0.7):
        messages = []

        if self.system_prompt:
            messages.append({"role": "system", "content": self.system_prompt})

        messages.append({"role": "user", "content": user_input})

        data = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
            "stream": False,
        }

        try:
            response = requests.post(
                self.api_url, headers=self.headers, json=data, timeout=120
            )

            response.raise_for_status()

            response_json = response.json()

            return response_json["choices"][0]["message"]["content"]

        except requests.exceptions.RequestException as e:
            print(f"Ollama API request failed: {e}")
            if "response" in locals() and response is not None:
                print(f"Error details: {response.text}")
            return None
