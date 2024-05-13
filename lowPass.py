import numpy as np
import matplotlib.pyplot as plt

# Circuit parameters
R = 4700  # Resistor value in ohms
C = 47e-9  # Capacitor value in farads
Vin = 10  # Input voltage in volts

# Frequency range for analysis
frequencies = np.logspace(1, 5, num=10)  # From 10 Hz to 100 kHz

# Calculate the output voltage for each frequency
Xc = 1 / (2 * np.pi * frequencies * C)
Vout = Vin * Xc / (Xc + R)

# Plot the frequency response
plt.figure(figsize=(8, 6))

# Pass band is before the corner freq
# once curve hits corner frequency, this area is called stop band
# in the stop band the curve is decreased at a slope of -20 in this example 
plt.semilogx(frequencies, 20 * np.log10(Vout / Vin), label="Low-Pass Filter Response")
plt.axhline(-3, color="red", linestyle="--", label="-3 dB cutoff")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Gain (dB)")
plt.title("RC Low-Pass Filter Frequency Response")
plt.grid(True)
plt.legend()
plt.show()
