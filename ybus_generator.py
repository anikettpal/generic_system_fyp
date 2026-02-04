import numpy as np

def get_system_data():
    """
    Collects Grid Topology and Load/Generation Data.
    Returns: Y_bus matrix, bus_data_list
    """
    print("--- STEP 1: System Data Entry ---")
    
    try:
        num_buses = int(input("Enter the total number of buses: "))
    except ValueError:
        return None, None

    # Initialize structures
    Y_bus = np.zeros((num_buses, num_buses), dtype=complex)
    bus_data = [] # List to store dictionaries of bus info

    print(f"\n--- Bus Data (Types & Setpoints) ---")
    print("Types: 1=Slack, 2=PV, 3=PQ")
    
    for i in range(num_buses):
        bid = i + 1
        print(f"\nSetting up Bus {bid}...")
        
        # Get Type
        while True:
            try:
                b_type_str = input(f"  Type (Slack/PV/PQ): ").strip().upper()
                if b_type_str == 'SLACK': type_code = 1
                elif b_type_str == 'PV': type_code = 2
                elif b_type_str == 'PQ': type_code = 3
                else: raise ValueError
                break
            except ValueError:
                print("  Invalid. Enter Slack, PV, or PQ.")

        # Get Voltages and Power
        # Note: Enter values in Per Unit (pu)
        V = float(input(f"  Voltage Magnitude V (pu) [Typ. 1.0]: "))
        theta = float(input(f"  Voltage Angle theta (deg) [Typ. 0.0]: "))
        Pg = float(input(f"  Active Power Generated Pg (pu): "))
        Pl = float(input(f"  Active Power Load Pl (pu): "))
        Qg = float(input(f"  Reactive Power Generated Qg (pu): "))
        Ql = float(input(f"  Reactive Power Load Ql (pu): "))
        
        # Net Power Injection (Generation - Load)
        P_spec = Pg - Pl
        Q_spec = Qg - Ql

        # Store in list (converting theta to radians)
        bus_data.append({
            'id': bid,
            'type': type_code,
            'V': V,
            'theta': np.radians(theta),
            'P_spec': P_spec,
            'Q_spec': Q_spec
        })

    print(f"\n--- Line Data Entry ---")
    print("Type 'done' when finished.")
    
    while True:
        conn = input("Enter connection (e.g., 1-2) or 'done': ").strip()
        if conn.lower() == 'done': break
            
        try:
            parts = conn.split('-')
            f_bus, t_bus = int(parts[0]), int(parts[1])
            
            r = float(input(f"  R (pu): "))
            x = float(input(f"  X (pu): "))
            b_in = input(f"  Half-line Charging B (pu) [Enter for 0]: ")
            b = float(b_in) if b_in else 0.0

            z = complex(r, x)
            y_s = 1/z
            y_sh = complex(0, b) # Assuming user enters B/2 or simplified B

            i, j = f_bus - 1, t_bus - 1
            
            Y_bus[i, j] -= y_s
            Y_bus[j, i] -= y_s
            Y_bus[i, i] += (y_s + y_sh)
            Y_bus[j, j] += (y_s + y_sh)
            
        except ValueError:
            print("Invalid input.")

    return Y_bus, bus_data