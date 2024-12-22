class BaseAction:
    def __init__(self):
        self.actions = []

    def execute(self):
        for action in self.actions: action.execute()        
    
    def stop(self):
        pass

    def wait(self, time: int):

