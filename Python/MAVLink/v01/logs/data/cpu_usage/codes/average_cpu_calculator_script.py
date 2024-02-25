import os
import re

def extract_cpu_usage(file_path):
    with open(file_path, 'r') as file:
        log_data = file.read()

    # Use regular expressions to find relevant information
    pattern = re.compile(r'(\d+) function calls in (\d+\.\d+) seconds')
    matches = pattern.findall(log_data)

    sum_average_call = 0
    for match in matches:
        ncall = int(match[0])
        timecall = float(match[1]) * 1_000_000
        average_call = timecall / ncall
        sum_average_call += average_call

    # Calculate the average time per function call in microseconds
    avg_time_per_call = sum_average_call / len(matches)
    avg_time_per_call = round(avg_time_per_call, 3)

    return avg_time_per_call

def process_files(parent_directory):
    results = []

    for filename in os.listdir(parent_directory):
        if filename.endswith('.txt'):  # Assuming your log files have a .txt extension
            log_file_path = os.path.join(parent_directory, filename)
            cpu_usage = extract_cpu_usage(log_file_path)

            results.append((filename, cpu_usage))

    return results

def save_to_file(results, output_file='average_cpu.txt'):
    with open(output_file, 'a') as file:
        for result in results:
            file.write(f"{result[0]} {result[1]}\n")

# Specify the parent directory where the log files are located
parent_directory = '../'

# Process log files and calculate average CPU usage
cpu_results = process_files(parent_directory)

# Write results to average_cpu.txt
save_to_file(cpu_results, 'average_cpu.txt')
