class FlipFlipCount:
    input: bool
    counter: int
    output: bool
    counter_target: int

    def __init__(self, counter_target: int):
        self.input = False
        self.counter = 0
        self.output = False
        self.counter_target = counter_target

    def update_input(self, input_bool: bool):
        self.input = input_bool
        self.counter += 1 if input_bool else 0
        self.output = True if self.counter == self.counter_target else False
