This program is a digital-twin simulator for a diesel generator, built with Windows Forms (.NET C#). It generates synthetic signals for multiple sensors acoustic, vibration, temperature, oil-quality, and RPM,using mathematical models instead of physical hardware. Each signal is displayed in the time domain and frequency domain using FFT, allowing users to observe engine characteristics such as vibration patterns, harmonic components, and noise behavior.
The system also performs pole–zero analysis in both S-domain and Z-domain to estimate the generator’s dynamic stability. All sensor outputs and stability indicators are combined into an integrated diagnostic overview that reflects the machine’s simulated health condition.

How to Use the Program

1. Launch the application
The main interface will show sensor panels, charts, control sliders, and the diagnostic overview.

2. Adjust sensor parameters
Use the sliders in each sensor panel to modify values such as vibration amplitude, acoustic pressure, temperature, permittivity, or pulse count for RPM.
Each change immediately updates the simulated signal.

3. Observe time-domain signals
The left chart in each panel shows the raw waveform generated from the selected parameter settings.

4. Analyze frequency components
The right chart shows the FFT spectrum, allowing you to see dominant frequencies, harmonics, or changes caused by different sensor conditions.

5. View stability analysis
The pole–zero graph updates based on vibration intensity, RPM, and oil-quality indicators, showing how stable or unstable the simulated generator becomes.

6. Check the diagnostic overview
The summary panel displays the combined interpretation of all sensors, providing a quick assessment of the simulated machine’s health.
