import copy
import ybus_generator
import nr_solver
import line_parameters

def check_limits(b_data, l_data, V_sol, Th_sol, Y_bus):
    """Checks for Base Case voltage and line flow violations."""
    violations = []
    
    # 1. Check Voltage Limits (0.95 to 1.05 pu standard)
    for i, b in enumerate(b_data):
        if V_sol[i] < 0.95:
            violations.append(f"Bus {b['id']} Undervoltage ({V_sol[i]:.3f} pu)")
        elif V_sol[i] > 1.05:
            violations.append(f"Bus {b['id']} Overvoltage ({V_sol[i]:.3f} pu)")

    # 2. Check Line Flow Limits
    bus_id_map = {b['id']: i for i, b in enumerate(b_data)}
    for line in l_data:
        if line['from'] not in bus_id_map or line['to'] not in bus_id_map:
            continue
            
        # Extract properties
        line_kV = line.get('voltage_kV', 230.0) 
        c_name = line_parameters.select_conductor(line_kV)
        I_max = line_parameters.conductors[c_name]['Imax']
        
        c_name, I_a, T_c, S_g, T_max = line_parameters.calculate_dynamic_line_state(
            line, V_sol, Th_sol, Y_bus, bus_id_map
        )
        
        if I_a > I_max:
            violations.append(f"Line {line['from']}-{line['to']} Overload ({I_a:.1f}A > {I_max}A)")
            
    return violations

def evaluate_security(b_data, l_data, system_freq, original_load):
    """Runs N-1 Contingency Analysis and categorizes Security Level."""
    
    # --- 1. BASE CASE CHECK ---
    current_load = sum(b['Pl'] for b in b_data if b['type'] == 3)
    load_lost = original_load - current_load > 0.0001  # Has UFLS fired?
    
    Y_bus = ybus_generator.build_y_bus(b_data, l_data)
    V_sol, Th_sol, P_cal, Q_cal = nr_solver.run_load_flow(Y_bus, b_data, system_freq, time_step=0)
    
    if V_sol is None:
        return "Level 5 (Noncorrectable Emergency)", ["Base Case Voltage Collapse"]
        
    base_violations = check_limits(b_data, l_data, V_sol, Th_sol, Y_bus)
    
    if load_lost:
        return "Level 6 (Restorative)", ["Loss of load has been suffered (UFLS Active)"]
    elif len(base_violations) > 0:
        return "Level 4/5 (Emergency)", base_violations

    # --- 2. N-1 CONTINGENCY CHECK (If Base Case is Secure) ---
    contingency_alarms = []
    
    # Simulate Line Outages (i^th line)
    for i, line in enumerate(l_data):
        l_cont = copy.deepcopy(l_data)
        l_cont.pop(i)  # Trip line
        
        Y_cont = ybus_generator.build_y_bus(b_data, l_cont)
        Vc, Thc, Pc, Qc = nr_solver.run_load_flow(Y_cont, b_data, system_freq)
        
        if Vc is None:
            contingency_alarms.append(f"Line {line['from']}-{line['to']} Outage -> Voltage Collapse")
        else:
            cv = check_limits(b_data, l_cont, Vc, Thc, Y_cont)
            if cv: contingency_alarms.append(f"Line {line['from']}-{line['to']} Outage -> Limits Violated")

    # Simulate Unit Outages (n^th unit - checking PV buses)
    for b in b_data:
        if b['type'] == 2:
            b_cont = copy.deepcopy(b_data)
            b_cont = [bus for bus in b_cont if bus['id'] != b['id']]
            l_cont = [l for l in l_data if l['from'] != b['id'] and l['to'] != b['id']]
            
            Y_cont = ybus_generator.build_y_bus(b_cont, l_cont)
            Vc, Thc, Pc, Qc = nr_solver.run_load_flow(Y_cont, b_cont, system_freq)
            
            if Vc is None:
                contingency_alarms.append(f"Gen {b['id']} Outage -> Voltage Collapse")
            else:
                cv = check_limits(b_cont, l_cont, Vc, Thc, Y_cont)
                if cv: contingency_alarms.append(f"Gen {b['id']} Outage -> Limits Violated")

   # Categorize
    if len(contingency_alarms) > 0:
        # CAPACITY CHECK: Do the surviving generators have enough P_max to cover the load?
        total_p_max = sum([b.get('P_max', 0) for b in b_data if b['type'] in [1, 2]])
        
        if total_p_max > current_load:
            # We have the physical capacity to fix the violations
            return "Level 2 (Correctively Secure)", contingency_alarms
        else:
            # We do not have the capacity. UFLS will be forced to act.
            return "Level 3 (Alert - Insufficient Capacity)", contingency_alarms
    else:
        return "Level 1 (Secure)", []