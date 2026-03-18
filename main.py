import sys
import time
import numpy as np
import ybus_generator
import nr_solver
import automatic_generation_control
import line_parameters
import ufls_controller
import load_fluctuator
import contingency_analysis  # <--- NEW IMPORT

# --- COLOR CODES ---
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m" 
MAGENTA = "\033[95m" # For Contingency Analysis
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
    
    b_data, l_data = ybus_generator.get_user_input()
    if not b_data: sys.exit()

    # --- CONTROL & SAFETY TUNING ---
    agc_sys = automatic_generation_control.AGC(K_p=2.0, K_i=0.02) 
    ufls_sys = ufls_controller.UFLS() 
    fluctuator = load_fluctuator.LoadFluctuator(interval=5) 

    # --- BASE LOAD SNAPSHOT (For Level 6 Check) ---
    original_total_load = sum([b['Pl'] for b in b_data if b['type'] == 3])

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

    # --- TRANSMISSION LINE FAULT SELECTION ---
    target_fault_line_idx = None
    
    if l_data:
        print("\n" + "="*40)
        print("   SELECT TRANSMISSION LINE TO FAULT")
        print("         (Symmetrical Fault)")
        print("="*40)
        print(f"{'Idx':<5} {'From':<10} {'To':<10}")
        print("-" * 30)
        for idx, line in enumerate(l_data):
            print(f"{idx+1:<5} Bus {line['from']:<6} Bus {line['to']:<6}")
        print("-" * 30)
        print("Enter '0' to SKIP line fault.")
        
        while True:
            try:
                user_input = input("Enter Line Idx to fault (or 0): ").strip()
                user_choice = int(user_input)
                if user_choice == 0:
                    target_fault_line_idx = None
                    break
                elif 1 <= user_choice <= len(l_data):
                    target_fault_line_idx = user_choice - 1
                    break
            except ValueError: pass

    # --- SIMULATION MODE LOGIC ---
    is_gen_trip_active = (target_trip_id is not None)
    is_fault_active = (target_fault_line_idx is not None)
    # Exclusively lock frequency when there is ONLY a symmetrical fault
    lock_freq_for_fault = is_fault_active and not is_gen_trip_active

    Y_bus = ybus_generator.build_y_bus(b_data, l_data)

    print("\n--- Starting Simulation (t=1 to 60s) ---")
    print("Initializing Steady State...")
    V_sol, Th_sol, P_cal, Q_cal = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=0)

    slack_idx = next(i for i, b in enumerate(b_data) if b['type'] == 1)
    current_turbine_power = P_cal[slack_idx] 

    for t in range(1, 61):
        print(f"\n{'='*20} t = {t} seconds {'='*20}")
        
        topology_changed = False
        
        if t == TRIP_TIME and target_trip_id is not None:
            print(f"!!! EVENT: GENERATOR AT BUS {target_trip_id} TRIPPED !!!")
            for b in b_data:
                if b['id'] == target_trip_id:
                    b['Pg'] = 0.0
                    b['P_spec'] = b['Pg'] - b['Pl']
                    if b['type'] == 2:
                        b['type'] = 3  
            target_trip_id = None

        if t == TRIP_TIME and target_fault_line_idx is not None:
            faulted_line = l_data[target_fault_line_idx]
            print(f"!!! EVENT: SYMMETRICAL FAULT ON LINE {faulted_line['from']}-{faulted_line['to']} !!!")
            
            bus_id_map = {b['id']: i for i, b in enumerate(b_data)}
            I_sc_pu, I_sc_amps = line_parameters.calculate_short_circuit_current(
                faulted_line, V_sol, Th_sol, Y_bus, bus_id_map
            )
            
            if I_sc_amps is not None:
                print(f"-> 3-Phase Short Circuit Current at Bus {faulted_line['from']}: {RED}{I_sc_amps:.2f} A{RESET} ({I_sc_pu:.2f} pu)")
            else:
                print("-> Short Circuit Current could not be computed (singular matrix).")
                
            print("-> Fault cleared by tripping the line.")
            l_data.pop(target_fault_line_idx)
            topology_changed = True
            target_fault_line_idx = None

        if topology_changed:
            Y_bus = ybus_generator.build_y_bus(b_data, l_data)
            print("-> Grid Topology Updated.")

        # Fluctuate load (Simulates random demand variations)
        fluctuated, alert = fluctuator.fluctuate_load(t, b_data)
        if fluctuated:
            print(f"{CYAN}{alert}{RESET}")

        V_sol, Th_sol, P_calc, Q_calc = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=t)
        
        if V_sol is None:
            print("Simulation Crash (Voltage Collapse).")
            break

        bus_id_map = {b['id']: i for i, b in enumerate(b_data)}

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

        print(f"\n[ PHYSICAL STATE - LINES ]")
        print(f"{'Line':<8} {'Cond':<10} {'Current(A)':<12} {'Temp(C)':<10} {'Sag(m)':<8}")
        
        for line in l_data:
            if line['from'] not in bus_id_map or line['to'] not in bus_id_map:
                continue 
                
            c_name, I_a, T_c, S_g, T_max = line_parameters.calculate_dynamic_line_state(
                line, V_sol, Th_sol, Y_bus, bus_id_map
            )
            limit_color = RED if T_c > T_max else RESET
            print(f"{line['from']}-{line['to']:<6} {c_name:<10} {I_a:<12.2f} {limit_color}{T_c:<10.2f}{RESET} {S_g:<8.2f}")


# --- PERIODIC CONTINGENCY ANALYSIS ---
        if t % 10 == 0:
            print(f"\n{MAGENTA}[ CONTINGENCY SWEEP ]{RESET}")
            level, alarms = contingency_analysis.evaluate_security(b_data, l_data, SYSTEM_FREQ, original_total_load)
            
            print(f"{MAGENTA}Security Status: {level}{RESET}") # <--- ADD THIS LINE BACK
            
            if alarms:
                for alarm in alarms:
                    print(f"   {MAGENTA}-> {alarm}{RESET}")
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

        if lock_freq_for_fault:
            net_imbalance = 0.0
            current_turbine_power = slack_p_demand 
        
        rocof = 0.0
        if abs(net_imbalance) > 0.000001:
            # Using Per-Unit Swing Equation
            rocof = (net_imbalance * 50.0) / (2 * H_CONST)
            SYSTEM_FREQ += rocof * TIME_STEP

        # Apply Under Frequency Load Shedding
        shed_occurred, ufls_alerts = ufls_sys.check_and_shed(t, SYSTEM_FREQ, rocof, b_data)
        if shed_occurred:
            for alert in ufls_alerts:
                print(f"{YELLOW}{alert}{RESET}")
            print(f"{YELLOW} -> Load reduced. NR Solver target updated.{RESET}")
        
        print(f"Turbine Output: {current_turbine_power:.4f} pu (Target: {target_mech_power:.4f})")
        print(f"AGC Output:     {p_agc:.4f} pu")
        print(f"Frequency:      {SYSTEM_FREQ:.4f} Hz | RoCoF: {rocof:.4f} Hz/s")

        for i in range(len(b_data)):
            b_data[i]['V'] = V_sol[i]
            b_data[i]['theta'] = Th_sol[i]

        time.sleep(0.05) 

if __name__ == "__main__":
    main()