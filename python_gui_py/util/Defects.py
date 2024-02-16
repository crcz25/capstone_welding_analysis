import json
from datetime import datetime

import numpy as np


class NPEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, datetime):
            return obj.isoformat()
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super(NPEncoder, self).default(obj)


class WeldDefectsReport:
    def __init__(self):
        self.report = {"weld_defects": [{"type": "Defect type", "defects": []}]}
        self.defect_types = {}

    def add_defect(self, defect_type, defect_id, timestamp, quality, height, limit, x_pos):
        # Check if the defect type already exists
        if defect_type not in self.defect_types:
            # If not, add it to the defects list and keep track of its position
            defect_entry = {"name": defect_type, "defects_found": []}
            self.report["weld_defects"][0]["defects"].append(defect_entry)
            self.defect_types[defect_type] = (
                len(self.report["weld_defects"][0]["defects"]) - 1
            )

        # Add the defect to the report
        defect_index = self.defect_types[defect_type]
        self.report["weld_defects"][0]["defects"][defect_index]["defects_found"].append(
            {
                "profile": defect_id,
                "timestamp": timestamp,
                "quality": quality,
                "height": height,
                "height_limit": limit,
                "x_position": x_pos,
            }
        )

    def serialize(self):
        return json.dumps(self.report, indent=4, cls=NPEncoder)


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
                return f"{grade}", threshold
        return "No Excessive defect", "NA"


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
                return f"{grade}", threshold
        return "No Sagging defect", "NA"


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

    # Example usage
    report = WeldDefectsReport()

    # Simulate adding defects
    report.add_defect("Excessive", 1, "2023-01-01T12:34:56Z", "B")
    report.add_defect("Excessive", 2, "2023-02-01T12:34:56Z", "C")
    report.add_defect("Sagging", 1, "2023-01-01T12:34:56Z", "D")

    # Serialize to JSON
    json_output = report.serialize()

    print(json_output)


if __name__ == "__main__":
    main()
