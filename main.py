import sys
import time
import cmath
import numpy as np
import ybus_generator
import nr_solver
import automatic_generation_control
import transmission_line_physics

# --- COLOR CODES ---
RED = "\033[91m"
RESET = "\033[0m"

# --- SIMULATION PARAMETERS ---
SYSTEM_FREQ = 50.0   
H_CONST = 5.0        
TIME_STEP = 1.0      
TRIP_TIME = 5

# --- PHYSICS CONSTANTS ---
DAMPING = 0.02       
GOV_LAG = 0.4        
TURB_LAG = 0.2       

# --- SYSTEM BASE VALUES ---
BASE_MVA = 100.0
BASE_KV  = 230.0    
I_BASE   = (BASE_MVA * 1e6) / (np.sqrt(3) * BASE_KV * 1e3) 

def main():
    global SYSTEM_FREQ
    
    b_data, l_data = ybus_generator.get_user_input()
    if not b_data: sys.exit()

    agc_sys = automatic_generation_control.AGC(K_p=2.0, K_i=0.02) 

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

    Y_bus = ybus_generator.build_y_bus(b_data, l_data)

    print("\n--- Starting Simulation (t=1 to 60s) ---")
    print(f"Base Voltage: {BASE_KV} kV | Base Current: {I_BASE:.2f} A")
    print("Initializing Steady State...")
    V_sol, Th_sol, P_cal, Q_cal = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=0)

    slack_idx = next(i for i, b in enumerate(b_data) if b['type'] == 1)
    current_turbine_power = P_cal[slack_idx] 
    current_valve_position = P_cal[slack_idx]

    # --- SIMULATION LOOP ---
    for t in range(1, 61):
        print(f"\n{'='*20} t = {t} seconds {'='*20}")
        
        if t == TRIP_TIME and target_trip_id is not None:
            print(f"!!! EVENT: BUS {target_trip_id} TRIPPED !!!")
            b_data = [b for b in b_data if b['id'] != target_trip_id]
            Y_bus = ybus_generator.build_y_bus(b_data, l_data)
            print("-> Grid Topology Updated.")
            target_trip_id = None

        V_sol, Th_sol, P_calc, Q_calc = nr_solver.run_load_flow(Y_bus, b_data, SYSTEM_FREQ, time_step=t)
        
        if V_sol is None:
            print("Simulation Crash (Voltage Collapse).")
            break

        # --- DISPLAY BUS DATA ---
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
                if p_val > p_max + 0.0001: p_str = f"{RED}{p_max:.4f}{RESET}"
            print(f"{b['id']:<4} {V_sol[i]:<10.4f} {deg:<10.4f} {p_str:<18} {Q_calc[i]:<10.4f}")

        # --- DISPLAY LINE PHYSICS (USING YOUR FORMULAS) ---
        print("-" * 88)
        print(f"{'Line':<8} {'Current(A)':<12} {'Cond.':<8} {'Load%':<8} {'Temp(C)':<10} {'Sag(m)':<8} {'Len(km)':<8}")
        
        id_map = {b['id']: i for i, b in enumerate(b_data)}

        for line in l_data:
            if line['from'] in id_map and line['to'] in id_map:
                i = id_map[line['from']]
                j = id_map[line['to']]
                
                # I = |(Vi - Vj) * y|
                z = complex(line['r'], line['x'])
                y = 1/z
                v_i = cmath.rect(V_sol[i], Th_sol[i])
                v_j = cmath.rect(V_sol[j], Th_sol[j])
                i_pu = abs((v_i - v_j) * y) 
                
                i_amps = i_pu * I_BASE
                
                # --- USE USER INPUT LENGTH ---
                line_len_km = line.get('length', 50.0) # Uses your input, defaults to 50 if missing
                
                c_name, load_pct, temp, sag, imax = transmission_line_physics.get_line_status(BASE_KV, i_amps, line_len_km)
                
                load_str = f"{load_pct:.1f}%"
                if load_pct > 100: load_str = f"{RED}{load_str}{RESET}"
                
                line_name = f"{line['from']}-{line['to']}"
                print(f"{line_name:<8} {i_amps:<12.1f} {c_name:<8} {load_str:<8} {temp:<10.2f} {sag:<8.2f} {line_len_km:<8.1f}")
        print("-" * 88)


        # --- DYNAMICS ---
        raw_agc = agc_sys.calculate_regulation(SYSTEM_FREQ, TIME_STEP)
        AGC_LIMIT = 0.5
        if raw_agc > AGC_LIMIT: p_agc = AGC_LIMIT
        elif raw_agc < -AGC_LIMIT: p_agc = -AGC_LIMIT
        else: p_agc = raw_agc
        
        target_valve_pos = slack_p_demand + p_agc 
        if target_valve_pos > slack_p_limit: target_valve_pos = slack_p_limit

        diff_gov = target_valve_pos - current_valve_position
        current_valve_position += diff_gov * GOV_LAG

        diff_turb = current_valve_position - current_turbine_power
        current_turbine_power += diff_turb * TURB_LAG
        
        damping_loss = DAMPING * (SYSTEM_FREQ - 50.0)
        net_imbalance = current_turbine_power - slack_p_demand - damping_loss
        
        rocof = 0.0
        if abs(net_imbalance) > 0.000001:
            total_load_est = sum([b['Pl'] for b in b_data])
            numerator = net_imbalance * 50.0
            denominator = total_load_est * 2 * H_CONST
            rocof = numerator / denominator
            SYSTEM_FREQ += rocof * TIME_STEP
        
        print(f"   Valve: {current_valve_position:.4f} | Turb: {current_turbine_power:.4f} | Freq: {SYSTEM_FREQ:.4f} Hz")

        for i in range(len(b_data)):
            b_data[i]['V'] = V_sol[i]
            b_data[i]['theta'] = Th_sol[i]

        time.sleep(0.05)

if __name__ == "__main__":
    main()