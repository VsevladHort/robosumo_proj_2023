class ProgramStatus:
    def __init__(self):
        self.is_running = False

    def start(self):
        self.is_running = True

    def stop(self):
        self.is_running = False

    def is_program_running(self):
        return self.is_running
