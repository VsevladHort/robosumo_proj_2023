class ProgramStatus:
    def __init__(self):
        self.is_running = False

    def start(self):
        if not self.is_running:
            self.is_running = True
            print("Program is now running.")
        else:
            print("Program is already running.")

    def stop(self):
        if self.is_running:
            self.is_running = False
            print("Program has been stopped.")
        else:
            print("Program is not running.")

    def is_program_running(self):
        return self.is_running
