class ProgramStatus:
    def __init__(self):
        self.is_running = False

    def start(self):
        if not self.is_running:
            self.is_running = True

    def stop(self):
        if self.is_running:
            self.is_running = False

    def is_program_running(self):
        return self.is_running
