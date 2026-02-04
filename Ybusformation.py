import numpy as np

def generate_y_bus():
    print("--- Y-Bus Matrix Generator ---")
    
    # 1. Get the number of buses
    try:
        num_buses = int(input("Enter the total number of buses: "))
    except ValueError:
        print("Please enter a valid integer.")
        return

    # Initialize Y_bus with complex zeros
    Y_bus = np.zeros((num_buses, num_buses), dtype=complex)
    
    # Store bus types for display later
    bus_data = {}

    print(f"\n--- Bus Classification ---")
    print("Types: Slack (1), PV (2), PQ (3)")
    for i in range(num_buses):
        bus_id = i + 1
        while True:
            try:
                b_type = input(f"Bus {bus_id} Type (Slack/PV/PQ): ").strip().upper()
                if b_type in ['SLACK', 'PV', 'PQ']:
                    bus_data[bus_id] = b_type
                    break
                else:
                    print("Invalid input. Please type Slack, PV, or PQ.")
            except ValueError:
                continue

    print(f"\n--- Line Data Entry ---")
    print("Enter line connections. Type 'done' when finished.")
    
    while True:
        print("-" * 20)
        conn = input("Enter connection (e.g., 1-2) or 'done': ").strip()
        
        if conn.lower() == 'done':
            break
            
        try:
            # Parse the "1-2" format
            parts = conn.split('-')
            if len(parts) != 2:
                print("Invalid format. Use 'From-To' (e.g., 1-2)")
                continue
                
            from_bus = int(parts[0])
            to_bus = int(parts[1])
            
            # Validate bus numbers
            if not (1 <= from_bus <= num_buses) or not (1 <= to_bus <= num_buses):
                print(f"Error: Buses must be between 1 and {num_buses}")
                continue
                
            # Get Impedance Data
            r = float(input(f"  Enter Resistance R for line {from_bus}-{to_bus} (pu): "))
            x = float(input(f"  Enter Reactance X for line {from_bus}-{to_bus} (pu): "))
            
            # Ask for Line Charging
            b_charging = input(f"  Enter Total Line Charging Susceptance B (pu) [Press Enter for 0]: ")
            b_total = float(b_charging) if b_charging else 0.0

            # --- Calculate Admittance ---
            z = complex(r, x)
            if z == 0:
                print("  Error: Impedance cannot be zero.")
                continue
                
            y_series = 1 / z
            y_shunt = complex(0, b_total / 2) # Half charging at each end

            # Adjust indices (Python is 0-indexed, Power Systems are 1-indexed)
            i = from_bus - 1
            j = to_bus - 1

            # --- Update Y-Bus Matrix ---
            # Off-diagonal terms: Y_ij = -y_series
            # Note: We subtract because admittance is additive in parallel.
            # If multiple lines exist between same buses, this accumulates correctly.
            Y_bus[i, j] -= y_series
            Y_bus[j, i] -= y_series

            # Diagonal terms: Sum of admittances connected + shunt
            Y_bus[i, i] += (y_series + y_shunt)
            Y_bus[j, j] += (y_series + y_shunt)
            
            print(f"  -> Line {from_bus}-{to_bus} added.")

        except ValueError:
            print("Invalid number entered. Please try again.")

    # --- Output Results ---
    print("\n" + "="*30)
    print("       FINAL CONFIGURATION")
    print("="*30)
    
    # Print Bus Types
    print("Bus Types:")
    for b_id, b_type in bus_data.items():
        print(f"  Bus {b_id}: {b_type}")

    # Print Y-Bus with cleaner formatting
    print("\nCalculated Y-Bus Matrix:")
    
    rows, cols = Y_bus.shape
    for r in range(rows):
        row_str = "| "
        for c in range(cols):
            val = Y_bus[r, c]
            
            # Logic to handle signs cleanly
            real_part = val.real
            imag_part = val.imag
            
            # Determine sign for imaginary part
            if imag_part >= 0:
                sign = "+"
            else:
                sign = "-"
            
            # Use abs() for imaginary part so we don't get double negatives like "--"
            row_str += f"{real_part:7.4f} {sign} {abs(imag_part):6.4f}j  "
            
        print(row_str + "|")

if __name__ == "__main__":
    generate_y_bus()