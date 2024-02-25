def calculate_average_difference(file1, file2):
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        lines1 = [float(line.strip()) for line in f1.readlines()]
        lines2 = [float(line.strip()) for line in f2.readlines()]

        if len(lines1) != len(lines2):
            raise ValueError("Number of lines in the two files must be the same.")

        differences = [a - b for a, b in zip(lines1, lines2)]
        average_difference = sum(differences) / len(differences)

    return average_difference

if __name__ == "__main__":
    file1_path = "timestamp_log_gcs.log"
    file2_path = "timestamp_log_drone.log"

    try:
        result = calculate_average_difference(file1_path, file2_path)
        print(f"Average Difference: {result}")
    except ValueError as e:
        print(f"Error: {e}")
