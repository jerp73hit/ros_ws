import requests

class OpenAIChatClient:
    def __init__(self, api_key, system_prompt="", model="gpt-4o-mini"):
        """
        Initializes the OpenAI client.
        """
        self.api_key = api_key
        self.system_prompt = system_prompt
        self.model = model
        self.api_url = "https://api.openai.com/v1/chat/completions"
        
        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

    def update_system_prompt(self, new_prompt):
        """Allows updating the system prompt after initialization."""
        self.system_prompt = new_prompt

    def generate_response(self, user_input, temperature=0.7):
        """
        Sends the user input to the OpenAI API and returns the text response.
        """
        messages = []
        
        if self.system_prompt:
            messages.append({"role": "system", "content": self.system_prompt})
            
        messages.append({"role": "user", "content": user_input})

        data = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature
        }

        try
            response = requests.post(self.api_url, headers=self.headers, json=data)
            
            response.raise_for_status()
            
            response_json = response.json()
            
            return response_json['choices'][0]['message']['content']

        except requests.exceptions.RequestException as e:
            print(f"API Request failed: {e}")
            if 'response' in locals() and response is not None:
                print(f"Error details: {response.text}")
            return None

