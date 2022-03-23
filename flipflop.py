class FlipFlipCount:
    counter: int
    output: bool
    counter_target: int

    def __init__(self, counter_target: int):
        self.counter = 0
        self.output = False
        self.counter_target = counter_target

    def update_input(self, input_bool: bool):
        if self.output:
            self.output = False

        # Falling edge
        if self.counter >= self.counter_target and not input_bool:
            self.output = True
            self.counter = 0

        self.counter += 1 if input_bool else 0


    def falling_edge(self):
        return self.output
