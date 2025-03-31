from collections import deque

class AudioHistory:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AudioHistory, cls).__new__(cls)
            cls._instance.history = deque(maxlen=3)  # Store up to 3 audio files
        return cls._instance

    def add_audio(self, audio_file):
        """
        Add an audio file to the history.
        """
        self.history.append(audio_file)

    def get_history(self):
        """
        Get the list of recently played audio files.
        """
        return list(self.history)