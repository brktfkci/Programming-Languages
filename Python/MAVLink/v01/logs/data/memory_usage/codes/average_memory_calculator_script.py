import os
import re

def extract_memory_usage(log_file):
    memory_usage_pattern = re.compile(r'(\d+)\s+([\d.]+)\s+MiB\s+([\d.]+)\s+MiB\s+(\d+)\s+drone.send_spefic\(\)')

    with open(log_file, 'r') as file:
        lines = file.readlines()

    memory_usages = []

    for line in lines:
        match = memory_usage_pattern.search(line)
        if match:
            line_number, start_memory, increment_memory, occurrences = match.groups()
            memory_usages.append({
                'line_number': int(line_number),
                'start_memory': float(start_memory),
                'increment_memory': float(increment_memory),
                'occurrences': int(occurrences)
            })

    return memory_usages

# Specify the parent directory where the log files are located
parent_directory = '../'

# Create a list to store results for each log file
results = []

# Loop through each file in the directory
for filename in os.listdir(parent_directory):
    if filename.endswith('.txt'):  # Assuming your log files have a .txt extension
        log_file_path = os.path.join(parent_directory, filename)
        memory_usages = extract_memory_usage(log_file_path)

        if memory_usages:
            average_increment_memory = sum(usage['increment_memory'] for usage in memory_usages) / len(memory_usages)
            average_increment_memory_kb = round(average_increment_memory * 1024 * 1.024)

            results.append((filename, average_increment_memory_kb))

# Write results to output.txt
with open('avarage_memory.txt', 'a') as output_file:
    for result in results:
        output_file.write(f"{result[0]} {result[1]}\n")
