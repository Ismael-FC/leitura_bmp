import matplotlib.pyplot as plt
import csv

csv_filename = "float_log.csv"
num_series = 7

# Initialize plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
lines = [ax.plot([], [], 'o', label=f"Series {i + 1}")[0] for i in range(num_series)]

ax.set_xlabel("Sample Number")
ax.set_ylabel("Pressure Reading")
ax.set_title("Pressure Sensor Data (Manual Refresh)")
ax.grid(True)
ax.legend()
plt.tight_layout()
plt.show()

def load_and_update():
    # Load CSV data
    series = [[] for _ in range(num_series)]
    sample_indices = [[] for _ in range(num_series)]

    try:
        with open(csv_filename, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for sample_num, row in enumerate(reader):
                try:
                    values = [float(val) for val in row if val.strip() != '']
                    for i, value in enumerate(values):
                        if i < num_series:
                            series[i].append(value)
                            sample_indices[i].append(sample_num + 1)
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"File not found: {csv_filename}")
        return

    # Update the plot lines
    for i in range(num_series):
        lines[i].set_data(sample_indices[i], series[i])

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()

# Main loop
print("Press ENTER to refresh the plot. Ctrl+C to exit.")
while True:
    input()  # Wait for user input
    load_and_update()