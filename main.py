import sys
import time
import numpy as np
import ybus_generator
import nr_solver

# --- SIMULATION PARAMETERS ---
SYSTEM_FREQ = 50.0   # Hz
H_CONST = 5.0        # Inertia Constant (MW-s/MVA)
TIME_STEP = 1.0      # Seconds
R_DROOP = 0.05       # Governor Droop (5%)
                     # This means 5% freq drop causes 100% Power increase.
                     # K_gov = 1/R = 20 pu Power per 1 pu Freq.

def main():
    global SYSTEM_FREQ
    print("=========================================")
    print("  DYNAMIC CONTINGENCY SIMULATOR (QSTS)")
    print("  *With Primary Frequency Response*")
    print("=========================================")

    # 1. Get Initial Data
    b_data, l_data = ybus_generator.get_user_input()
    if not b_data: sys.exit()

    Y_bus = ybus_generator.build_y_bus(b_data, l_data)

    print("\n--- Starting Simulation (t=1 to 14s) ---")
    
    # We need to track the Mechanical Power of the Slack Bus separately.
    # Initially, Mechanical Power = Electrical Power (Steady State).
    # We need to run one initial load flow to find this starting point.
    print("Initializing System State...")
    V_init, Th_init = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=0)
    
    # Find Slack Bus and store its initial Mechanical Power
    slack_bus_idx = next(i for i, b in enumerate(b_data) if b['type'] == 1)
    
    # Calculate initial Slack P (we have to estimate it from the load flow P_calc)
    # But for simplicity, we assume P_mech starts roughly equal to total load.
    # A better way: The slack bus P in the last Load Flow is our P_mech_initial.
    # Note: nr_solver prints it but doesn't return P_calc. 
    # Let's approximate: P_mech = Total Load - Other Gens.
    total_load_init = sum([b['Pl'] for b in b_data])
    total_gen_other = sum([b['Pg'] for b in b_data if b['type'] == 2])
    p_mech_slack = total_load_init - total_gen_other
    
    print(f"Initial Slack Mechanical Power: {p_mech_slack:.4f} pu")

    # This variable tracks the "Missing Power" in the system
    net_imbalance = 0.0
    
    for t in range(1, 15):
        
        # --- EVENT TRIGGER: Trip FIRST PV Bus at t=5 ---
        if t == 5:
            bus_to_remove = next((b for b in b_data if b['type'] == 2), None)
            
            if bus_to_remove:
                trip_id = bus_to_remove['id']
                print(f"\n!!! EVENT: PV BUS {trip_id} TRIPPED !!!")
                
                # 1. The Disturbance (Step Change)
                # We lost a generator, so P_mech_total dropped instantly.
                p_lost = bus_to_remove['P_spec'] 
                
                # Remove from data
                b_data = [b for b in b_data if b['id'] != trip_id]
                Y_bus = ybus_generator.build_y_bus(b_data, l_data)
                print("   -> Grid Topology Updated.")
            else:
                p_lost = 0

        # --- GOVERNOR RESPONSE (The Fix) ---
        # 1. Calculate Governor Response based on Frequency Deviation
        # deviation = (50 - current_freq) / 50  (in per unit)
        freq_dev_pu = (50.0 - SYSTEM_FREQ) / 50.0
        
        # Power boost = Deviation / Droop
        # If freq is low (positive dev), we ADD power.
        p_governor_boost = freq_dev_pu / R_DROOP
        
        # Update Mechanical Power (Valve opening)
        current_p_mech_slack = p_mech_slack + p_governor_boost
        
        # 2. Calculate Imbalance (Dynamic Update)
        # Imbalance = Total Mechanical Supply - Total Electrical Load
        # Note: If we just tripped a gen, 'p_lost' is gone from supply side.
        
        # We need the NEW total load (sum of remaining buses)
        current_total_load = sum([b['Pl'] for b in b_data])
        
        # Net Imbalance = (Slack_Mech + Remaining_PV_Mech) - Load
        # (Assuming other PVs are constant for now)
        current_p_mech_others = sum([b['P_spec'] for b in b_data if b['type'] == 2])
        
        total_supply = current_p_mech_slack + current_p_mech_others
        net_imbalance = total_supply - current_total_load
        
        # Debug Prints
        if t >= 5:
            print(f"   [Gov Control] Freq: {SYSTEM_FREQ:.2f} | Dev: {freq_dev_pu:.4f} pu")
            print(f"   [Gov Control] Slack P_mech increases by: {p_governor_boost:.4f} pu")
            print(f"   [Net Balance] Supply {total_supply:.4f} - Load {current_total_load:.4f} = Imbalance {net_imbalance:.4f}")

        # --- PHYSICS: Calculate Frequency Change ---
        if abs(net_imbalance) > 0.0001:
            # RoCoF = (Imbalance * f0) / (2 * H * Total_Load)
            # Imbalance is negative if we are short on power.
            numerator = net_imbalance * 50.0
            denominator = current_total_load * 2 * H_CONST
            rocof = numerator / denominator
            
            SYSTEM_FREQ += rocof * TIME_STEP
            print(f"   -> RoCoF: {rocof:.4f} Hz/s | New Freq: {SYSTEM_FREQ:.4f} Hz")

        # --- RUN SOLVER ---
        V_sol, Th_sol = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=t)
        
        if V_sol is None:
            print("Simulation Crash.")
            break
            
        # Warm Start Update
        for i in range(len(b_data)):
            b_data[i]['V'] = V_sol[i]
            b_data[i]['theta'] = Th_sol[i]

        time.sleep(0.5)

if __name__ == "__main__":
    main()