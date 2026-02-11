import sys
import time
import numpy as np
import ybus_generator
import nr_solver

# --- SIMULATION PARAMETERS ---
SYSTEM_FREQ = 50.0   # Hz
H_CONST = 5.0        # Inertia Constant
TIME_STEP = 1.0      # Seconds
TRIP_TIME = 5        # The exact second the bus trips

def main():
    global SYSTEM_FREQ
    
    # 1. Get Initial Data
    b_data, l_data = ybus_generator.get_user_input()
    if not b_data: sys.exit()

    Y_bus = ybus_generator.build_y_bus(b_data, l_data)

    print("\n--- Starting Simulation (t=1 to 14s) ---")
    
    # Initialize System (Steady State t=0)
    print("Initializing Steady State...")
    V_sol, Th_sol = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=0)

    for t in range(1, 15):
        # --- HEADER ---
        print(f"\n{'='*20} t = {t} seconds {'='*20}")
        
        # --- EVENT LOGIC ---
        if t == TRIP_TIME:
            bus_to_remove = next((b for b in b_data if b['type'] == 2), None)
            if bus_to_remove:
                trip_id = bus_to_remove['id']
                print(f"!!! EVENT: PV BUS {trip_id} TRIPPED !!!")
                b_data = [b for b in b_data if b['id'] != trip_id]
                Y_bus = ybus_generator.build_y_bus(b_data, l_data)
                print("-> Grid Topology Updated.")

        # --- 1. RUN LOAD FLOW ---
        V_sol, Th_sol = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=t)
        
        if V_sol is None:
            print("Simulation Crash (Voltage Collapse).")
            break

        # Calculate Slack Bus Output
        P_slack_elec_demand = 0.0
        # Slack is Index 0 (Bus 1)
        for k in range(len(b_data)):
            mag_Y = abs(Y_bus[0, k])
            ang_Y = np.angle(Y_bus[0, k])
            P_slack_elec_demand += V_sol[0] * V_sol[k] * mag_Y * np.cos(Th_sol[0] - Th_sol[k] - ang_Y)

        # --- 2. INSTANT RAMP UP ---
        current_p_mech = P_slack_elec_demand
        
        # --- 3. PHYSICS ---
        net_imbalance = current_p_mech - P_slack_elec_demand
        rocof = 0.0

        if abs(net_imbalance) > 0.000001:
            total_load_est = sum([b['Pl'] for b in b_data])
            numerator = net_imbalance * 50.0
            denominator = total_load_est * 2 * H_CONST
            rocof = numerator / denominator
            SYSTEM_FREQ += rocof * TIME_STEP
        
        # --- DISPLAY ---
        print(f"Frequency: {SYSTEM_FREQ:.4f} Hz | RoCoF: {rocof:.4f} Hz/s")

        # Update Data
        for i in range(len(b_data)):
            b_data[i]['V'] = V_sol[i]
            b_data[i]['theta'] = Th_sol[i]

        time.sleep(0.5)

if __name__ == "__main__":
    main()