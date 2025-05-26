class SubModule:
    def __init__(self, environment):
        self.environment = environment
        self.case_dir = environment.case_dir
        self.config = environment.config