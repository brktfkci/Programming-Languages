import matplotlib.pyplot as plt

def read_data(file_path):
    categories = []
    values = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():
                category, value = line.split()
                categories.append(category)
                values.append(float(value))

    return categories, values

def plot_data(categories, values, title, ylabel, save_path=None):
    # A list of colors for each category
    colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'pink']

    plt.bar(categories, values, color=colors[:len(categories)])
    plt.title(title, fontsize=15, pad=15)
    plt.ylabel(ylabel, fontsize=15, labelpad=15)
    # plt.xlabel('Categories', fontsize=15)
    plt.xticks(rotation=45, ha='right', fontsize=15)  # Rotate x labels for better visibility
    plt.yticks(fontsize=15)
    
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')


if __name__ == "__main__":
    file_path_time = "../data/time_usage/codes/average_times.txt"  
    file_path_memory = "../data/memory_usage/codes/average_memory.txt"  
    file_path_cpu = "../data/cpu_usage/codes/average_cpu.txt"  

    categories_time, values_time = read_data(file_path_time)
    categories_memory, values_memory = read_data(file_path_memory)
    categories_cpu, values_cpu = read_data(file_path_cpu)
    
    plot_data(categories_time, values_time, "Execution Time", "Time (us)", save_path="time_consumption_plot.png")
    plt.clf() 
    plot_data(categories_memory, values_memory, "Memory Consumption", "Memory (KB)", save_path="memory_consumption_plot.png")
    plt.clf() 
    plot_data(categories_cpu, values_cpu, "CPU Access Time Interval", "Time Interval (us) for CPU Access", save_path="cpu_usage_plot.png")
    plt.clf() 
