import numpy as np

# B-G431B-ESC1 Parameter
Vref = 3.3
R60 = 4.7e3
R25 = 10e3
B = 3950
T25 = 298.15
ADC_MAX = 4095

# Temperaturpunkte: -10 bis 140°C, 10°C Schritte
temps_c = np.arange(-10, 150, 10)
temps_k = temps_c + 273.15

# RT1(T) → U_ADC → ADC_raw = U_ADC * ADC_MAX / Vref
RT1 = R25 * np.exp(B * (1/temps_k - 1/T25))
U_ADC = Vref * R60 / (R60 + RT1)
ADC_raw = np.round(U_ADC / Vref * ADC_MAX).astype(int)

# C-Array: ADC_raw → Temperatur
print("const uint16_t adc_lut_raw[16] = {")
for i, adc in enumerate(ADC_raw):
    print(f"  {adc:4d}", end='' if i<15 else '')
    print(", " if i<15 else '')
print("\n};")

# Verifikation
print("\n# T[°C] → ADC_raw[0-4095]")
for t, adc in zip(temps_c, ADC_raw):
    print(f"T={t:3d}°C → ADC={adc:4d}")

