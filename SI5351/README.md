Code for drivers using the SI5351 clock frequency synthesizer.

Current command set:

stop<br>
start<br>
sweep<br>
ICset<br>
freq1<br>
freq2<br>
freq3<br>
duty<br>
res<br>
ns<br>
out<br>
bifreq<br>
swstart<br>
swstop<br>
swstep<br>
swrate<br>
count<br>
init<br>
LED<br>
check<br>
ID<br>

stop    - Stops all signals and routines.

start   - Begin transmitting signals using current settings and output.

sweep   - Start a sweep routine, according to current settings.

ICset   - Set all three frequencies at once.

freq1   - Set the frequency for output 1, decimal numbers accepted, in Hz.  Output signal with be half the set frequency.

freq2   - Set the frequency for output 2, decimal numbers accepted, in Hz.  Output signal with be half the set frequency.

freq3   - Set the frequency for output 3, decimal numbers accepted, in Hz.  Output signal with be half the set frequency.

duty    - Set the output pulse duty cycle as a decimal number between .05 and .95, computed relative to the current output frequency.

res     - Directly set the resistance of the digipot which controls signal pulse width.

ns      - Directly set the target pulse time for signal pulse width.

out     - Select which frequency is linked to the output; 1, 2, or 3.

bifreq  - Toggle bi-frequency mode.

swstart - Set the low/starting frequency for a sweep.

swstop  - Set the high/stopping frequency for a sweep.

swstep  - Set the frequency step size for a sweep.

swrate  - Set the number of frequency steps per second, decimal numbers accepted, as Hz.

count   - Set the number of pulses executed per start command.  0 results in continuous mode operation.

init    - Re-initialize the SI5351

LED     - Toggle LED blinking on the microcontroller.

check   - Prints the current settings to serial.

ID      - Print the ID of the microcontroller to serial.
