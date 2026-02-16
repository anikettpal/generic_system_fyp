import numpy as np

# --- 1. USER'S EXACT DATA & FORMULAS ---

Tamb = 25   # ambient temperature
wind_speed = 3   # m/s

conductors = {
    "Zebra":   {"R20": 0.068, "alpha": 0.004, "weight": 1.62,  "span": 300, "tension": 20000, "Imax": 1100, "Tmax": 100},
    "Panther": {"R20": 0.139, "alpha": 0.004, "weight": 0.86,  "span": 80,  "tension": 9000,  "Imax": 450,  "Tmax": 75},
    "Dog":     {"R20": 0.272, "alpha": 0.004, "weight": 0.394, "span": 60,  "tension": 5000,  "Imax": 300,  "Tmax": 75}
}

def select_conductor(V):
    if V >= 200:
        return "Zebra"
    elif V >= 30:
        return "Panther"
    else:
        return "Dog"

def conductor_temp(I, Rline):
    h_wind = 1 + 0.6 * wind_speed
    return Tamb + (I**2 * Rline * 1e-3) / h_wind

def sag(weight, span, tension):
    w = weight * 9.81    # N/m
    # Using your exact return formula:
    return (weight * span**2) / (2 * tension)

def max_power(V, Imax):
    return np.sqrt(3) * V * Imax / 1000

# --- 2. INTERFACE FOR MAIN.PY ---

def get_line_status(voltage_kV, current_A, length_km):
    """
    Calculates the status using the user's formulas.
    Returns: (ConductorName, Loading%, Temp, Sag)
    """
    # 1. Select Conductor
    cname = select_conductor(voltage_kV)
    c = conductors[cname]

    # 2. Calculate Parameters
    Rline = c["R20"] * length_km
    Tc = conductor_temp(current_A, Rline)
    Sag = sag(c["weight"], c["span"], c["tension"])
    Loading = 100 * current_A / c["Imax"]
    
    return cname, Loading, Tc, Sag, c["Imax"]