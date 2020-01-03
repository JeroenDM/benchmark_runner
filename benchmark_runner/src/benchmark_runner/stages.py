
class Container:
    def __init__(self, stages):
        self.stages = stages
        self.planning_graph = []

    def shedule(self):
        pass


class Stage(object):
    def __init__(self):
        self.left_nb = None
        self.right_nb = None


class Generator(Stage):
    def __init__(self):
        pass


class Propagator(Stage):
    pass


class Connector(Stage):
    pass
