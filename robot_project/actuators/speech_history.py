from collections import deque

class SpeechHistory:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SpeechHistory, cls).__new__(cls)
            cls._instance.history = deque(maxlen=6)  # Store up to 3 texts
        return cls._instance

    def add_text(self, text):
        """
        Add a text to the history.
        """
        self.history.append(text)

    def get_history(self):
        """
        Get the list of recently received texts.
        """
        return list(self.history)