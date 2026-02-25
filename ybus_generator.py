import numpy as np

def get_user_input():
    print("--- STEP 1: System Data Entry ---")
    try:
        num_buses = int(input("Enter the total number of buses: "))
    except ValueError:
        return None, None

    bus_data = []
    print(f"\n--- Bus Data ---")
    print("Types: 1=Slack, 2=PV, 3=PQ")
    
    for i in range(num_buses):
        bid = i + 1
        print(f"\nSetting up Bus {bid}...")
        while True:
            try:
                type_code = int(input("  Type (1=Slack, 2=PV, 3=PQ): "))
                if type_code in [1, 2, 3]: break
            except ValueError: pass
        
        # --- NEW: Ask for Limits for Generators ---
        p_max_limit = 999.99 # Default high value for PQ
        if type_code == 1 or type_code == 2:
            p_max_limit = float(input(f"  GENERATOR LIMIT: Max Power P_max (pu): "))

        V = float(input(f"  Voltage V (pu): "))
        theta = float(input(f"  Angle theta (deg): "))
        Pg = float(input(f"  Gen Pg (pu): "))
        Pl = float(input(f"  Load Pl (pu): "))
        Qg = float(input(f"  Gen Qg (pu): "))
        Ql = float(input(f"  Load Ql (pu): "))
        
        bus_data.append({
            'id': bid,
            'type': type_code,
            'V': V,
            'theta': np.radians(theta),
            'Pg': Pg, 'Pl': Pl, 'Qg': Qg, 'Ql': Ql,
            'P_spec': Pg - Pl,
            'Q_spec': Qg - Ql,
            'P_max': p_max_limit  # Store the limit
        })

    line_data = []
    print(f"\n--- Line Data (Type 'done' to finish) ---")
    while True:
        conn = input("Enter connection (e.g., 1-2) or 'done': ").strip()
        if conn.lower() == 'done': break
        try:
            parts = conn.split('-')
            line_data.append({
                'from': int(parts[0]),
                'to': int(parts[1]),
                'r': float(input("  R (pu): ")),
                'x': float(input("  X (pu): ")),
                'b': float(input("  Half-line B (pu): ") or 0.0),
                'N': int(0),
            })
        except ValueError:
            print("Invalid input.")
            
    while True:
        conn = input("Enter lines with transformers (e.g., 1-2) or 'done': ").strip()
        if conn.lower() == 'done': break
        try:
            parts = conn.split('-')
            line_data.append({
                'from': int(parts[0]),
                'to': int(parts[1]),
                'N': int(input("  Turns Ratio N : ")),
                'rt': float(input("  R of transformer (pu): ")),
                'xt': float(input("  X of transformer (pu): ")),   
            })
        except ValueError:
            print("Invalid input.")         
    return bus_data, line_data

def build_y_bus(bus_data, line_data):
    num_buses = len(bus_data)
    id_map = {b['id']: i for i, b in enumerate(bus_data)}
    
    Y = np.zeros((num_buses, num_buses), dtype=complex)
    
    for line in line_data:
        if line['from'] in id_map and line['to'] in id_map:
            i = id_map[line['from']]
            j = id_map[line['to']]            
            z = complex(line['r'], line['x'])
            a = line['N']
            y_s = 1/z
           
            y_sh = complex(0, line['b'])
            
            Y[i, j] -= y_s
            Y[j, i] -= y_s
            Y[i, i] += (y_s + y_sh)
            Y[j, j] += (y_s + y_sh)
            
            if a != 0:
             zt = complex(line['rt'], line['xt'])
             y_t = 1/zt
             Y[i, j] -= y_t/a
             Y[j, i] -= y_t/a
             Y[i, i] += y_t/a**2
             Y[j, j] += y_t
            
    return Y