import numpy as np

def get_user_input():
    """
    Asks user for data ONCE at the start.
    Returns: list of buses, list of lines
    """
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
            'Q_spec': Qg - Ql
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
                'b': float(input("  Half-line B (pu): ") or 0.0)
            })
        except ValueError:
            print("Invalid input.")

    return bus_data, line_data

def build_y_bus(bus_data, line_data):
    """
    Pure Math Function. Takes data lists and returns Y_bus matrix.
    Can be called repeatedly when topology changes.
    """
    num_buses = len(bus_data)
    # Map real Bus IDs to Matrix Indices (0, 1, 2...)
    # This is crucial if we delete Bus 2, and Bus 3 becomes index 1.
    id_map = {b['id']: i for i, b in enumerate(bus_data)}
    
    Y = np.zeros((num_buses, num_buses), dtype=complex)
    
    for line in line_data:
        # Check if both ends of the line still exist in the bus list
        if line['from'] in id_map and line['to'] in id_map:
            i = id_map[line['from']]
            j = id_map[line['to']]
            
            z = complex(line['r'], line['x'])
            y_s = 1/z
            y_sh = complex(0, line['b'])
            
            Y[i, j] -= y_s
            Y[j, i] -= y_s
            Y[i, i] += (y_s + y_sh)
            Y[j, j] += (y_s + y_sh)
            
    return Y