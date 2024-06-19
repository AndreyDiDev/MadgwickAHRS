import matplotlib.pyplot as pyplot
import numpy

# Calculate angle and position
SAMPLE_RATE = 100
ANGULAR_VELOCITY = 45
RADIUS = 1

starting_stationary = numpy.zeros(2 * SAMPLE_RATE)  # 2 seconds
accelerating = numpy.linspace(0, ANGULAR_VELOCITY, 5 * SAMPLE_RATE)  # 5 seconds
constant_velocity = numpy.full(3 * SAMPLE_RATE, ANGULAR_VELOCITY)  # 3 seconds
decelerating = numpy.linspace(ANGULAR_VELOCITY, 0, 5 * SAMPLE_RATE)  # 5 seconds
ending_stationary = numpy.zeros(2 * SAMPLE_RATE)  # 2 seconds

omega = numpy.concatenate((starting_stationary, accelerating, constant_velocity, decelerating, ending_stationary))

time = numpy.linspace(0, (len(omega) - 1) / SAMPLE_RATE, len(omega))

theta = (numpy.cumsum(omega) / SAMPLE_RATE) % 360

x = RADIUS * numpy.cos(numpy.radians(theta))
y = RADIUS * numpy.sin(numpy.radians(theta))

# Write to file
with open("circle.csv", "w") as file:
    file.write("Time (s),X (m),Y (m),Z (m),X (deg),Y (deg),Z (deg)\n")

    for index, _ in enumerate(time):
        file.write(str(time[index]) + "," + str(x[index]) + "," + str(y[index]) + ",0,0,0," + str(theta[index]) + "\n")

# Plot
_, axes = pyplot.subplots(nrows=4, sharex=True)

axes[0].plot(time, omega, label="omega")
axes[0].set_ylabel("Angular velocity (°/s)")
axes[0].grid()
axes[0].legend()

axes[1].plot(time, theta, label="theta")
axes[1].set_ylabel("Angle (°)")
axes[1].grid()
axes[1].legend()

axes[2].plot(time, x, label="x")
axes[2].plot(time, y, label="y")
axes[2].set_ylabel("Position (m)")
axes[2].set_xlabel("Time (s)")
axes[2].grid()
axes[2].legend()

data = numpy.genfromtxt("C:/Users/Andrey/Documents/AHRSRepo/MadgwickAHRS/circle1.txt", delimiter=",", skip_header=1)

acceleration = data[:, 13:16]

def integrate_acceleration(initial_position, initial_velocity, acceleration_values, dt):
    x, v = initial_position, initial_velocity
    positions = [x]

    for a in acceleration_values:
        # Update velocity
        v += a * dt

        # Update position
        x += v * dt
        positions.append(x)

    return positions

positions = numpy.empty((len(acceleration), 2))

initial_x = 0.0  # Initial position
initial_v = 0.0  # Initial velocity
# acceleration_values = [2.0, 1.5, 3.0, 2.5]  # Replace with your actual acceleration data
time_step = 0.01  # Time step (in seconds)

final_positions = integrate_acceleration(initial_x, initial_v, acceleration[:,0], time_step)
final_positionsY = integrate_acceleration(initial_x, initial_v, acceleration[:,1], time_step)


# for index in range(len(acceleration)):
#     position[index] = integrate_accelerations(acceleration[0], acceleration[1], data[index, 0])

print(final_positions)

# axes[3].scatter(x, y, color='b', label='Data points')
axes[3].plot(x, y, linestyle='--', color='r', label='Line connecting points')
axes[3].plot(final_positions, final_positionsY)

axes[3].set_ylabel('x')
axes[3].set_xlabel('y')
# axes[3].set_title('Scatter Plot of x vs. y')
axes[3].grid(True)
axes[3].legend()

pyplot.show()
