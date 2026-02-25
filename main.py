import sys
import time
import numpy as np
import ybus_generator
import nr_solver
import automatic_generation_control
import line_parameters
import ufls_controller  # <--- UFLS Module

# --- COLOR CODES ---
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"

# --- SIMULATION PARAMETERS ---
SYSTEM_FREQ = 50.0   
H_CONST = 5.0        
TIME_STEP = 1.0      
TRIP_TIME = 5

# --- PHYSICS CONSTANTS ---
DAMPING = 0.02       
TURBINE_LAG = 0.3    

def main():
    global SYSTEM_FREQ
    
    # 1. Get Initial Data
    b_data, l_data = ybus_generator.get_user_input()
    if not b_data: sys.exit()

    # --- CONTROL & SAFETY TUNING ---
    agc_sys = automatic_generation_control.AGC(K_p=2.0, K_i=0.02) 
    ufls_sys = ufls_controller.UFLS() # Initialize UFLS Relay & CSV logger

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
        print("Enter '0' to SKIP tripping.")
        
        while True:
            try:
                user_input = input("Enter Bus ID to trip (or 0): ").strip()
                user_choice = int(user_input)
                if user_choice == 0:
                    target_trip_id = None
                    break
                elif user_choice in [b['id'] for b in pv_buses]:
                    target_trip_id = user_choice
                    break
            except ValueError: pass

    # Build Y-Bus
    Y_bus = ybus_generator.build_y_bus(b_data, l_data)

    print("\n--- Starting Simulation (t=1 to 60s) ---")
    print("Initializing Steady State...")
    V_sol, Th_sol, P_cal, Q_cal = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=0)

    # Initialize Turbine and Dynamics
    slack_idx = next(i for i, b in enumerate(b_data) if b['type'] == 1)
    current_turbine_power = P_cal[slack_idx] 
    rocof = 0.0  # Initialized here so the UFLS can log it at t=1

    # --- SIMULATION LOOP (60 Seconds) ---
    for t in range(1, 61):
        print(f"\n{'='*25} t = {t} seconds {'='*25}")
        
        # --- EVENT LOGIC ---
        if t == TRIP_TIME and target_trip_id is not None:
            print(f"{RED}!!! EVENT: BUS {target_trip_id} TRIPPED !!!{RESET}")
            b_data = [b for b in b_data if b['id'] != target_trip_id]
            Y_bus = ybus_generator.build_y_bus(b_data, l_data)
            print("-> Grid Topology Updated.")
            target_trip_id = None

        # --- UFLS LOGIC ---
        # Checks frequency, sheds load if needed, and logs state to CSV
        shed_occurred, ufls_alerts = ufls_sys.check_and_shed(t, SYSTEM_FREQ, rocof, b_data)
        if shed_occurred:
            for alert in ufls_alerts:
                print(f"{YELLOW}{alert}{RESET}")
            print(f"{YELLOW}   -> Load reduced. NR Solver target updated.{RESET}")

        # --- 1. RUN LOAD FLOW ---
        V_sol, Th_sol, P_calc, Q_calc = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=t)
        
        if V_sol is None:
            print(f"{RED}Simulation Crash (Voltage Collapse).{RESET}")
            break

        # Map IDs to matrix indices for the line_parameters module
        bus_id_map = {b['id']: i for i, b in enumerate(b_data)}

        # --- 2. DISPLAY ELECTRICAL TABLE ---
        print(f"\n[ ELECTRICAL STATE ]")
        print(f"{'ID':<4} {'V (pu)':<10} {'Ang (deg)':<10} {'P (pu)':<10} {'Q (pu)':<10}")
        
        slack_p_demand = 0.0
        slack_p_limit = 999.0
        
        for i, b in enumerate(b_data):
            deg = np.degrees(Th_sol[i])
            p_val = P_calc[i]
            p_str = f"{p_val:.4f}"
            
            if b['type'] == 1:
                slack_p_demand = p_val
                slack_p_limit = b.get('P_max', 999.0)
            
            if b['type'] in [1, 2]:
                p_max = b.get('P_max', 999.0)
                if p_val > p_max + 0.0001:
                    p_str = f"{RED}{p_max:.4f}{RESET}"
            
            display_p = p_str if RED not in p_str else p_str
            print(f"{b['id']:<4} {V_sol[i]:<10.4f} {deg:<10.4f} {display_p:<18} {Q_calc[i]:<10.4f}")


        # --- 3. DISPLAY PHYSICAL TABLE (LINES) ---
        print(f"\n[ PHYSICAL STATE - LINES ]")
        print(f"{'Line':<8} {'Cond':<10} {'Current(A)':<12} {'Temp(C)':<10} {'Sag(m)':<8}")
        
        for line in l_data:
            if line['from'] not in bus_id_map or line['to'] not in bus_id_map:
                continue 
            
            if 'voltage_kV' not in line: line['voltage_kV'] = 230.0
            if 'length_km' not in line: line['length_km'] = line.get('length', 50.0)
                
            c_name, I_a, T_c, S_g, T_max = line_parameters.calculate_dynamic_line_state(
                line, V_sol, Th_sol, Y_bus, bus_id_map
            )
            
            limit_color = RED if T_c > T_max else RESET
            line_name = f"{line['from']}-{line['to']}"
            print(f"{line_name:<8} {c_name:<10} {I_a:<12.2f} {limit_color}{T_c:<10.2f}{RESET} {S_g:<8.2f}")


        # --- 4. DYNAMICS & CONTROL ---
        print(f"\n[ GRID CONTROL ]")
        
        raw_agc = agc_sys.calculate_regulation(SYSTEM_FREQ, TIME_STEP)
        
        AGC_LIMIT = 0.5
        if raw_agc > AGC_LIMIT: p_agc = AGC_LIMIT
        elif raw_agc < -AGC_LIMIT: p_agc = -AGC_LIMIT
        else: p_agc = raw_agc
        
        target_mech_power = slack_p_demand + p_agc
        
        if target_mech_power > slack_p_limit: target_mech_power = slack_p_limit

        diff = target_mech_power - current_turbine_power
        current_turbine_power += diff * TURBINE_LAG
        
        damping_loss = DAMPING * (SYSTEM_FREQ - 50.0)
        net_imbalance = current_turbine_power - slack_p_demand - damping_loss
        
        rocof = 0.0
        if abs(net_imbalance) > 0.000001:
            total_load_est = sum([b['Pl'] for b in b_data]) # <--- Automatically uses reduced load if UFLS fired
            if total_load_est > 0:
                numerator = net_imbalance * 50.0
                denominator = total_load_est * 2 * H_CONST
                rocof = numerator / denominator
                SYSTEM_FREQ += rocof * TIME_STEP
        
        print(f"   Turbine Output: {current_turbine_power:.4f} pu (Target: {target_mech_power:.4f})")
        print(f"   AGC Output:     {p_agc:.4f} pu")
        print(f"   Frequency:      {SYSTEM_FREQ:.4f} Hz | RoCoF: {rocof:.4f} Hz/s")

        for i in range(len(b_data)):
            b_data[i]['V'] = V_sol[i]
            b_data[i]['theta'] = Th_sol[i]

        time.sleep(0.05) 

if __name__ == "__main__":
    main()