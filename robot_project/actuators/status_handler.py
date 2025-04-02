import queue

class StatusHandler:
    def __init__(self):
        self.command_queue = None  # Initialize the command queue as None

    def set_command_queue(self, command_queue):
        """
        Set the command queue to be used for sending status updates.
        """
        self.command_queue = command_queue

    def update_status(self, status):
        """
        Send a status update to the GUI via the command queue.
        """
        if self.command_queue:
            self.command_queue.put({"type": "status_update", "status": status})
        else:
            print("[STATUS_HANDLER] Command queue is not set. Cannot send status update.")

# Create a global instance of StatusHandler
status_handler = StatusHandler()