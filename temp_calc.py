#!/usr/bin/env python3
"""
Temporäre Berechnungen
"""

# Deine Berechnungen hier

import numpy as np

# Parameter für B-G431B-ESC1 NTC
R25 = 10e3  # Ohm bei 25°C
B = 3950    # Beta-Wert [K]
T25 = 298.15  # 25°C in Kelvin

# Temperaturpunkte: -10°C bis 140°C, 10°C Schritte (16 Punkte)
temps_c = np.arange(-10, 150, 10)  # [-10, 0, 10, ..., 140]
temps_k = temps_c + 273.15

# Beta-Gleichung
resistances = R25 * np.exp(B * (1/temps_k - 1/T25)) / 1000 * 100  # kOhm*100

# C-Array Format ausgeben
print("const uint16_t r_lut[16] = {")
for i, r in enumerate(np.round(resistances).astype(int)):
    print(f"  {r:4d}", end='' if i<15 else '')
    print(", " if i<15 else '')
print("\n};")

# Tabelle zur Kontrolle
print("\n# Verifikation:")
for t, r in zip(temps_c, resistances):
    print(f"T={t:3d}°C → R={r:6.1f} → uint16_t={int(round(r))}")
