class WeldDefect:
    def __init__(self, work_piece_thickness):
        self.work_piece_thickness = work_piece_thickness
        self.criteria = {}

    def evaluate_defect(self, height_of_weld):
        raise NotImplementedError("Subclasses must implement evaluate method")


class Excessive(WeldDefect):
    def __init__(self, work_piece_thickness):
        super().__init__(work_piece_thickness)
        self.criteria = {
            "D": min(0.4 + 0.4 * self.work_piece_thickness, 10),
            "C": min(0.4 + 0.3 * self.work_piece_thickness, 10),
            "B": min(0.4 + 0.2 * self.work_piece_thickness, 10),
        }

    def evaluate_defect(self, height_of_weld):
        for grade, threshold in self.criteria.items():
            if height_of_weld > threshold:
                return f"Excessive - {grade} class"
        return "No Excessive defect"


class Sagging(WeldDefect):
    def __init__(self, work_piece_thickness):
        super().__init__(work_piece_thickness)
        self.criteria = {
            "D": min(0.3 * self.work_piece_thickness, 3),
            "C": min(0.2 * self.work_piece_thickness, 2),
            "B": min(0.1 * self.work_piece_thickness, 1),
        }

    def evaluate_defect(self, height_of_weld):
        for grade, threshold in self.criteria.items():
            if height_of_weld > threshold:
                return f"Sagging - {grade} class"
        return "No Sagging defect"


class WeldDefectFactory:
    @staticmethod
    def get_defect_evaluator(defect_type, work_piece_thickness):
        if defect_type == "Excessive":
            return Excessive(work_piece_thickness)
        elif defect_type == "Sagging":
            return Sagging(work_piece_thickness)
        else:
            raise ValueError("Unknown defect type")


def find_defect(defect_choice, work_piece_thickness, height_of_weld):
    evaluator = WeldDefectFactory.get_defect_evaluator(
        defect_choice, work_piece_thickness
    )
    result = evaluator.evaluate_defect(height_of_weld)
    return result


def main():
    # Example usage:
    defect_choice = "Excessive"  # or "Sagging"
    work_piece_thickness = 5  # Example thickness
    height_of_weld = 1.0  # Example weld height measuring from thickness to max of weld

    # Find defects
    find_defect(defect_choice, work_piece_thickness, height_of_weld)


if __name__ == "__main__":
    main()
