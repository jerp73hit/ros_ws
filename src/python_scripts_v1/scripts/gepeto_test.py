from gepeto import OpenAIChatClient
import os

API_KEY = os.getenv("OPENAI_API_KEY")

SYSTEM = ""

ai_client = OpenAIChatClient(
    api_key=API_KEY, 
    system_prompt=SYSTEM
)

user_text = ""

result = ai_client.generate_response(user_text)

if result:
    print("Success! Passing this to the next function...")
    print(result)
else:
    print("Something went wrong with the API call.")

