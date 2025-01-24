import sys
import os
from collections import defaultdict

def get_size(obj, seen=None):
    """Calculates the memory size of an object, avoiding circular references."""
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0  # Avoid circular references
    seen.add(obj_id)
    size = sys.getsizeof(obj)
    if isinstance(obj, dict):
        size += sum(get_size(k, seen) + get_size(v, seen) for k, v in obj.items())
    elif hasattr(obj, '__dict__'):
        size += get_size(vars(obj), seen)
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum(get_size(i, seen) for i in obj)
    return size

def memory_report(obj, filename="memory_report.txt"):
    """
    Generates a memory usage report for an object (one layer deep) and saves it to a .txt file.
    
    :param obj: The object to analyze.
    :param filename: The name of the output .txt file (default: 'memory_report.txt').
    """
    def classify_objects(obj):
        """Classifies objects into parent classes and their attributes."""
        class_map = defaultdict(list)
        for attr_name in dir(obj):
            try:
                attr = getattr(obj, attr_name)
                if not callable(attr) and not attr_name.startswith("__"):
                    class_map[type(attr)].append(attr)
            except Exception:
                continue
        return class_map

    # Classify objects and calculate sizes
    class_map = classify_objects(obj)
    total_memory = 0
    report_lines = []

    report_lines.append(f"Memory usage report for object of type {type(obj).__name__}")
    report_lines.append("=" * 60)
    report_lines.append("Aggregate memory usage by parent class:")
    
    for cls, instances in class_map.items():
        cls_total = sum(get_size(instance) for instance in instances)
        total_memory += cls_total
        report_lines.append(f"{cls.__name__}: {cls_total / 1024:.2f} KB")
        for instance in instances:
            try:
                instance_size = get_size(instance)
                report_lines.append(f"  Instance ({repr(instance)[:50]}): {instance_size / 1024:.2f} KB")
            except RecursionError:
                report_lines.append(f"  Instance ({repr(instance)[:50]}): Unable to calculate size (recursion limit reached)")

    report_lines.append("=" * 60)
    report_lines.append(f"Total memory usage: {total_memory / 1024:.2f} KB")

    # Write the report to a .txt file
    file_path = os.path.join(os.getcwd(), filename)
    with open(file_path, "w") as report_file:
        report_file.write("\n".join(report_lines))

    print(f"Memory report saved to {file_path}")

# Example usage
if __name__ == "__main__":
    class Parent:
        def __init__(self):
            self.data = [1] * 10**3

    class Child(Parent):
        def __init__(self):
            super().__init__()
            self.extra_data = [2] * 10**4
            self.circular_ref = self  # Add a circular reference

    parent_instance = Parent()
    child_instance = Child()

    memory_report(locals())  # Analyze all objects in local scope
