import sys
import time
import numpy as np
import ybus_generator
import nr_solver

# --- COLOR CODES ---
RED = "\033[91m"
RESET = "\033[0m"

# --- SIMULATION PARAMETERS ---
SYSTEM_FREQ = 50.0   
H_CONST = 5.0        
TIME_STEP = 1.0      
TRIP_TIME = 5        

def main():
    global SYSTEM_FREQ
    
    # 1. Get Initial Data
    b_data, l_data = ybus_generator.get_user_input()
    if not b_data: sys.exit()

    # --- PV BUS SELECTION ---
    pv_buses = [b for b in b_data if b['type'] == 2]
    target_trip_id = None
    
    if pv_buses:
        print("\n" + "="*40)
        print("      SELECT GENERATOR TO TRIP")
        print("="*40)
        print(f"{'ID':<5} {'Pg':<10} {'P_max':<10}")
        print("-" * 30)
        for b in pv_buses:
            print(f"{b['id']:<5} {b['Pg']:<10.4f} {b['P_max']:<10.4f}")
        print("-" * 30)
        
        while True:
            try:
                user_choice = int(input("Enter Bus ID to trip: "))
                if user_choice in [b['id'] for b in pv_buses]:
                    target_trip_id = user_choice
                    print(f"-> Target Confirmed: Bus {target_trip_id} will trip at t={TRIP_TIME}.")
                    break
                else: print("Invalid ID.")
            except ValueError: pass

    # Build Y-Bus
    Y_bus = ybus_generator.build_y_bus(b_data, l_data)

    print("\n--- Starting Simulation (t=1 to 14s) ---")
    print("Initializing Steady State...")
    # Initial run
    V_sol, Th_sol, P_cal, Q_cal = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=0)

    for t in range(1, 15):
        print(f"\n{'='*20} t = {t} seconds {'='*20}")
        
        # --- EVENT LOGIC ---
        if t == TRIP_TIME and target_trip_id is not None:
            print(f"!!! EVENT: BUS {target_trip_id} TRIPPED !!!")
            b_data = [b for b in b_data if b['id'] != target_trip_id]
            Y_bus = ybus_generator.build_y_bus(b_data, l_data)
            print("-> Grid Topology Updated.")
            target_trip_id = None 

        # --- 1. RUN LOAD FLOW ---
        V_sol, Th_sol, P_calc, Q_calc = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=t)
        
        if V_sol is None:
            print("Simulation Crash (Voltage Collapse).")
            break

        # --- 2. DISPLAY TABLE (WITH RED LIMITS) ---
        print(f"{'ID':<4} {'V (pu)':<10} {'Ang (deg)':<10} {'P (pu)':<10} {'Q (pu)':<10}")
        
        net_imbalance = 0.0
        
        for i, b in enumerate(b_data):
            deg = np.degrees(Th_sol[i])
            
            # Default values
            p_val = P_calc[i]
            p_str = f"{p_val:.4f}"
            
            # --- LIMIT CHECK LOGIC ---
            # Only apply to Slack(1) and PV(2)
            if b['type'] in [1, 2]:
                p_max = b['P_max']
                # If demand exceeds limit
                if p_val > p_max:
                    # We print the LIMIT in RED
                    p_str = f"{RED}{p_max:.4f}{RESET}"
                    
                    # Imbalance = Supply(Limit) - Demand(Calculated)
                    net_imbalance += (p_max - p_val)
                else:
                    # Normal print
                    p_str = f"{p_val:.4f}"
            
            # Note: We use length 20 for formatting p_str to account for invisible color codes
            # Adjust spacing if alignment looks off in your specific terminal
            print(f"{b['id']:<4} {V_sol[i]:<10.4f} {deg:<10.4f} {p_str:<18} {Q_calc[i]:<10.4f}")

        # --- 3. PHYSICS ---
        rocof = 0.0
        if abs(net_imbalance) > 0.000001:
            total_load_est = sum([b['Pl'] for b in b_data])
            numerator = net_imbalance * 50.0
            denominator = total_load_est * 2 * H_CONST
            rocof = numerator / denominator
            SYSTEM_FREQ += rocof * TIME_STEP
        
        print(f"Frequency: {SYSTEM_FREQ:.4f} Hz | RoCoF: {rocof:.4f} Hz/s")

        # Update Data
        for i in range(len(b_data)):
            b_data[i]['V'] = V_sol[i]
            b_data[i]['theta'] = Th_sol[i]

        time.sleep(0.5)

if __name__ == "__main__":
    main()