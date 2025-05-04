from collections import deque
import json, os

def load_scripts(json_path="scripts.json"):
    with open(json_path, "r") as f:
        return json.load(f)

def get_script_for_audio(file_path, scripts_dict):
    # Extract just the filename from the path
    file_name = os.path.basename(file_path)
    return scripts_dict.get(file_name, f"No script found for {file_name}")

class AudioHistory:
    _instance = None
    _scripts = None  # class-level cache of the scripts

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AudioHistory, cls).__new__(cls)
            cls._instance.history = deque(maxlen=3)
            if cls._scripts is None:
                with open("scripts.json", "r") as f:
                    cls._scripts = json.load(f)
        return cls._instance

    def add_audio(self, audio_file):
        """
        Add an audio file to the history.
        """
        scripts = load_scripts()
        line = get_script_for_audio(audio_file, scripts)
        self.history.append(line)

    def get_history(self):
        """
        Get the list of recently played audio files.
        """
        return list(self.history)