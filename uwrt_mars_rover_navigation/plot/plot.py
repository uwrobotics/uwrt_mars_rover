import matplotlib.pyplot as plt

def plot_distance_time(file_path):
    timestamps = []
    distances = []

    # Read data from the file
    with open(file_path, 'r') as file:
        for line in file:
            timestamp, distance = map(float, line.split())
            timestamps.append(timestamp)
            distances.append(distance)

    # Plot the data
    plt.plot(timestamps, distances, marker='o', linestyle='-')
    plt.title('Distance vs Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Distance (meters)')
    plt.grid(True)
    plt.show()

plot_distance_time('end.txt')
