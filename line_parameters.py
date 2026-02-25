import numpy as np

Tamb = 25  # ambient temperature
wind_speed = 3   # m/s

conductors = {
    "Zebra":{
        "R20":0.068, "alpha":0.004, "weight":1.62,
        "span":300, "tension":20000, "Imax":1100, "Tmax":100
    },
    "Panther":{
        "R20":0.139, "alpha":0.004, "weight":0.86,
        "span":80, "tension":9000, "Imax":450, "Tmax":75
    },
    "Dog":{
        "R20":0.272, "alpha":0.004, "weight":0.394,
        "span":60, "tension":5000, "Imax":300, "Tmax":75
    }
}

def select_conductor(V):
    if V >= 200:
        return "Zebra"
    elif V >= 30:
        return "Panther"
    else:
        return "Dog"

def conductor_temp(I, Rline):
    h_wind = 1 + 0.6*wind_speed
    return Tamb + (I**2 * Rline * 1e-3)/h_wind

def max_power(V, Imax):
    return np.sqrt(3)*V*Imax/1000

# --- UPDATED: Sag Function (Now includes Temperature) ---
def sag(weight, span, tension, Tc):
    # 1. Calculate Base Sag (at Ambient Temp)
    base_sag = (weight * span**2) / (2 * tension)
    
    # 2. Calculate Thermal Sag Increase
    if Tc > Tamb:
        alpha_thermal = 0.000019  # Physical stretch factor for ACSR
        delta_T = Tc - Tamb
        
        # Parabolic thermal stretch formula
        added_length_factor = (3 * span**2 * alpha_thermal * delta_T) / 8
        new_sag = np.sqrt(base_sag**2 + added_length_factor)
        return new_sag
        
    return base_sag

# --- UPDATED: Dynamic Integration Function ---
def calculate_dynamic_line_state(line, V_sol, Th_sol, Y_bus, bus_id_map, base_mva=100.0):
    """Takes Load Flow results and returns real-time physical line parameters."""
    # 1. Map IDs to matrix indices
    idx_i = bus_id_map[line['from']]
    idx_j = bus_id_map[line['to']]
    
    # 2. Reconstruct complex voltages
    Vi = V_sol[idx_i] * np.exp(1j * Th_sol[idx_i])
    Vj = V_sol[idx_j] * np.exp(1j * Th_sol[idx_j])
    
    # 3. Calculate Line Current Magnitude in pu
    y_ij = -Y_bus[idx_i, idx_j] 
    I_pu = np.abs((Vi - Vj) * y_ij)
    
    # 4. Convert pu current to actual Amperes
    line_kV = line.get('voltage_kV', 230.0) 
    line_km = line.get('length_km', 50.0)
    I_base = (base_mva * 1000) / (np.sqrt(3) * line_kV)
    I_amps = I_pu * I_base
    
    # 5. Get physical properties and calculate
    c_name = select_conductor(line_kV)
    c = conductors[c_name]
    R_total = c["R20"] * line_km
    
    # Calculate physical state
    temp = conductor_temp(I_amps, R_total)
    
    # Pass 'temp' into the sag calculation
    current_sag = sag(c["weight"], c["span"], c["tension"], temp)
    
    return c_name, I_amps, temp, current_sag, c["Tmax"]