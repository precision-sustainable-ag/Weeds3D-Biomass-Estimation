

class Evaluation:
    true_positives: int
    false_positives: int
    false_negatives: int

    def __init__(self, true_positives: int, false_positives: int, false_negatives: int):
        self.true_positives = true_positives
        self.false_positives = false_positives
        self.false_negatives = false_negatives

    def add(self, evaluation):
        self.true_positives += evaluation.true_positives
        self.false_positives += evaluation.false_positives
        self.false_negatives += evaluation.false_negatives

    def print(self):
        print("TP: "+str(self.true_positives)+", FP: "+str(self.false_positives)+", FN: "+str(self.false_negatives))