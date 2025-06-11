import os
import ollama

ollama_host = 'http://172.17.0.1:11434'
client = ollama.Client(host=ollama_host)

response = client.chat(
    model='llama3.2-vision:11b',
    messages=[
        {'role': 'user', 'content': 'What do you see in this image?'},
        {
            'role': 'user',
            'content': 'image attached',
            'images': ['path/to/image.jpg']
        }
    ]
)
