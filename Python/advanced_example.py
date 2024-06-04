
# print(PYTHON_INCLUDE_PATH)

# import sys 
# help('modules')
# print(sys.path)
import sys
# sys.path.append('''C:/Users/Andrey/Documents/AHRSRepo/MadgwickAHRS/Python''')

import imufusion
import matplotlib.pyplot as pyplot
import numpy

# Import sensor data
data = numpy.genfromtxt("C:/Users/Andrey/Documents/Fusion-main_new/Fusion-main/Fusion/Imu 1.csv", delimiter=",", skip_header=1)

sample_rate = 3  # 100 Hz

timestamp = data[:, 9]

for i in range(len(timestamp)):
    timestamp[i] = i * 1/3
    
gyroscope = data[:, 3:6]
accelerometer = data[:, 0:3]
magnetometer = data[:, 6:9]

print(numpy.shape(data))

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()



ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NED,
                                    0.5,  # gain
                                    245,  # gyroscope range
                                    10,  # acceleration rejection
                                    10,  # magnetic rejection
                                    5 * sample_rate)  # recovery trigger period = 5 seconds

# Process sensor data
print(timestamp[-2] - timestamp[-1])
delta_time = numpy.diff(timestamp, prepend=timestamp[0])
# delta_time = numpy.diff(timestamp)

euler = numpy.empty((len(timestamp), 3))
internal_states = numpy.empty((len(timestamp), 6))
flags = numpy.empty((len(timestamp), 4))

filteredEarthAcceleration = numpy.empty((len(timestamp), 1))
filteredLinearAcceleration = numpy.empty((len(timestamp), 1))

print(delta_time[:])
# initialize linear acceleration
# linearX = numpy.empty(len(3))
# linearY = numpy.empty(len(3))
# linearZ = numpy.empty(len(3))

for index in range(len(timestamp)):
    
    # print(ahrs.earth_acceleration)
    
    gyroscope[index] = offset.update(gyroscope[index]/1000)

    ahrs.update_no_magnetometer(gyroscope[index]/1000, accelerometer[index]/1000, 1/3)

    euler[index] = ahrs.quaternion.to_euler()
    
    filteredEarthAcceleration[index] = ahrs.earth_acceleration[2]
    filteredLinearAcceleration[index] = ahrs.linear_acceleration[2]

    ahrs_internal_states = ahrs.internal_states
    internal_states[index] = numpy.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored, #
                                          ahrs_internal_states.acceleration_recovery_trigger, #
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored, #
                                          ahrs_internal_states.magnetic_recovery_trigger]) #

    ahrs_flags = ahrs.flags
    flags[index] = numpy.array([ahrs_flags.initialising,
                                ahrs_flags.angular_rate_recovery,
                                ahrs_flags.acceleration_recovery,
                                ahrs_flags.magnetic_recovery])
    
    print(internal_states[index])
    print(flags[index])
    
    # linearX[index] = ahrs.getLinear
    
    



def plot_bool(axis, x, y, label):
    axis.plot(x, y, "tab:cyan", label=label)
    pyplot.sca(axis)
    pyplot.yticks([0, 1], ["False", "True"])
    axis.grid()
    axis.legend()


# Plot Euler angles
figure, axes = pyplot.subplots(nrows=11, sharex=True, gridspec_kw={"height_ratios": [6, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1]})

figure.suptitle("Euler angles, internal states, and flags")

axes[0].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[0].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[0].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[0].set_ylabel("Degrees")
axes[0].grid()
axes[0].legend()

# Plot initialising flag
plot_bool(axes[1], timestamp, flags[:, 0], "Initialising")

# Plot angular rate recovery flag
plot_bool(axes[2], timestamp, flags[:, 1], "Angular rate recovery")

# Plot acceleration rejection internal states and flag
axes[3].plot(timestamp, internal_states[:, 0], "tab:olive", label="Acceleration error")
axes[3].set_ylabel("Degrees")
axes[3].grid()
axes[3].legend()

plot_bool(axes[4], timestamp, internal_states[:, 1], "Accelerometer ignored")

axes[5].plot(timestamp, internal_states[:, 2], "tab:orange", label="Acceleration recovery trigger")
axes[5].grid()
axes[5].legend()

plot_bool(axes[6], timestamp, flags[:, 2], "Acceleration recovery")

# Plot magnetic rejection internal states and flag
axes[7].plot(timestamp, internal_states[:, 3], "tab:olive", label="Magnetic error")
axes[7].set_ylabel("Degrees")
axes[7].grid()
axes[7].legend()

plot_bool(axes[8], timestamp, internal_states[:, 4], "Magnetometer ignored")

axes[9].plot(timestamp, internal_states[:, 5], "tab:orange", label="Magnetic recovery trigger")
axes[9].grid()
axes[9].legend()

plot_bool(axes[10], timestamp, flags[:, 3], "Magnetic recovery")

pyplot.show(block="no_block" not in sys.argv)  # don't block when script run by CI

#########################################################################

# Create subplots
fig, (ax1, ax2) = pyplot.subplots(2, 1, figsize=(8, 6))

# Plot gyro data
ax1.plot(timestamp, gyroscope[:, 0], label='GyroX', color='red')
ax1.plot(timestamp, gyroscope[:, 1], label='GyroY', color='green')
ax1.plot(timestamp, gyroscope[:, 2], label='GyroZ', color='blue')
ax1.set_title('Gyro Data')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angular Velocity')
ax1.grid(True)
ax1.legend()

# Plot accelerometer data
ax2.plot(timestamp, accelerometer[:, 0], label='AccelerometerX', color='orange')
ax2.plot(timestamp, accelerometer[:, 1], label='AccelerometerY', color='purple')
ax2.plot(timestamp, accelerometer[:, 2], label='AccelerometerZ', color='brown')
ax2.set_title('Accelerometer Data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Acceleration')
ax2.grid(True)
ax2.legend()

pyplot.tight_layout()
pyplot.show()

#########################################################################
# import altimeter data
data1 = numpy.genfromtxt("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/Altimeter.csv", delimiter=",", skip_header=1)


time = data1[:, 0]
acceleration = data1[:, 1]

# Convert to m/s²
conversion_factor = 0.3048  # 1 ft/s² = 0.3048 m/s²
acceleration_m = acceleration

# Create subplots
fig, (ax1, ax2) = pyplot.subplots(2, 1, figsize=(8, 6), sharex=True)

# Plot data
ax1.plot(time, acceleration_m, color='red')
ax1.set_title('Alt Accel')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Acceleration')
ax1.grid(True)
ax1.legend()

# Plot accelerometer data
ax2.plot(timestamp, (accelerometer[:, 0]/1000) * (-9.81), label = "Raw", color='orange')
ax2.plot(timestamp, filteredEarthAcceleration * (-9.81), label = "Filtered", color='purple')
# ax2.plot(timestamp, filteredLinearAcceleration, label = "Linear", color='green')
ax2.set_title('Accel IMU')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Acceleration')
ax2.grid(True)
ax2.legend()

pyplot.tight_layout()
pyplot.show()

#########################################################################
dataBaro = numpy.genfromtxt("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/baro 3.csv", delimiter=",", skip_header=1)
# graph altitude 
height = data1[:, 3]

pressure = dataBaro[:, 0] # in Pa

seaLevelPressure = 1013.25 * 1000; # sea level pressure in hPa
altitudeBaro = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.190284))

# Convert to m
# altitudeBaro = 

# height_m = height * 0.3048

# Create subplots
fig, (ax1, ax2) = pyplot.subplots(2, 1, figsize=(8, 6), sharex=True)


ax1.plot(time, height, label = "Altimeter", color='red')
ax1.set_title('Alt height')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('meters')
ax1.grid(True)
ax1.legend()

timeS = timestamp[:543]

dataEverest = numpy.genfromtxt("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/IMU_BARO_ALT.csv", delimiter=",", skip_header=1)

i, j = 0, 0
new = numpy.empty((543, 1))

for i in range(len(new[:])):
    
    j = i * 10
    
    if(j < len(dataEverest[:])):
        new[i] = dataEverest[j, 11]
    
    # while(j < (i + 1) * 10) and (j < len(dataEverest[:])):
    #     sum  = dataEverest[j, 11]
    #     j = j + 1
    #     print(sum, i, j)
        
    # new[i] = sum / 10
    # print(new[i], i, j)
    
seaLevelPressure = 1013.25 * 1000; # sea level pressure in hPa
altitudeReal = 44330.0 * (1.0 - pow(new / seaLevelPressure, 0.190284))

ax2.plot(timeS, altitudeBaro * 0.3048, label = 'Baro', color='blue')
ax2.plot(timeS, altitudeReal * 0.3048, label = 'Everest', color='red')
ax2.set_title('Baro and Everest altitude')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('meters')
ax2.grid(True)
ax2.legend()

pyplot.tight_layout()
pyplot.show()
