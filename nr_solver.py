import numpy as np

def run_load_flow(Y_bus, bus_data, max_iter=20, tol=1e-5):
    print("\n--- STEP 2: Newton-Raphson Solver (Full) ---")
    
    num_buses = len(bus_data)
    
    # 1. Initialize State Variables from Input
    V = np.array([b['V'] for b in bus_data])
    Theta = np.array([b['theta'] for b in bus_data])
    
    # 2. Get Targets (Specified P and Q)
    P_spec = np.array([b['P_spec'] for b in bus_data])
    Q_spec = np.array([b['Q_spec'] for b in bus_data])
    types = np.array([b['type'] for b in bus_data]) # 1=Slack, 2=PV, 3=PQ

    # 3. Identify Indices
    idx_slack = np.where(types == 1)[0]
    idx_pv    = np.where(types == 2)[0]
    idx_pq    = np.where(types == 3)[0]
    
    non_slack = np.concatenate((idx_pv, idx_pq))
    non_slack.sort()
    
    # Main Iteration Loop
    for it in range(max_iter):
        # --- A. Calculate Power Injections ---
        P_calc = np.zeros(num_buses)
        Q_calc = np.zeros(num_buses)
        
        for i in range(num_buses):
            for k in range(num_buses):
                mag_Y = abs(Y_bus[i, k])
                ang_Y = np.angle(Y_bus[i, k])
                P_calc[i] += V[i] * V[k] * mag_Y * np.cos(Theta[i] - Theta[k] - ang_Y)
                Q_calc[i] += V[i] * V[k] * mag_Y * np.sin(Theta[i] - Theta[k] - ang_Y)

        # --- B. Calculate Mismatches ---
        dPa = P_spec - P_calc
        dQa = Q_spec - Q_calc
        
        mismatch = 0
        for i in non_slack:
            mismatch = max(mismatch, abs(dPa[i]))
        for i in idx_pq:
            mismatch = max(mismatch, abs(dQa[i]))
            
        print(f"\nIteration {it+1}: Max Mismatch = {mismatch:.6f}")
        
        if mismatch < tol:
            print(f"  -> Converged in {it+1} iterations.")
            break

        # --- C. Build Jacobian Matrix ---
        J1 = np.zeros((num_buses, num_buses))
        J2 = np.zeros((num_buses, num_buses))
        J3 = np.zeros((num_buses, num_buses))
        J4 = np.zeros((num_buses, num_buses))
        
        for i in range(num_buses):
            for k in range(num_buses):
                mag_Y = abs(Y_bus[i, k])
                ang_Y = np.angle(Y_bus[i, k])
                
                if i != k:
                    # Off-Diagonal Elements (FIXED SIGNS)
                    # We dropped the negative sign on J1 and J4 as discussed
                    term_a = V[i] * V[k] * mag_Y * np.sin(Theta[i] - Theta[k] - ang_Y)
                    term_b = V[i] * V[k] * mag_Y * np.cos(Theta[i] - Theta[k] - ang_Y)
                    
                    J1[i, k] = term_a      # dP/dTheta (Positive)
                    J2[i, k] = term_b / V[k]
                    J3[i, k] = -term_b
                    J4[i, k] = term_a / V[k] # dQ/dV (Positive)
                else:
                    # Diagonal Elements
                    J1[i, i] = -Q_calc[i] - (V[i]**2 * Y_bus[i,i].imag)
                    J2[i, i] = P_calc[i] / V[i] + (V[i] * Y_bus[i,i].real)
                    J3[i, i] = P_calc[i] - (V[i]**2 * Y_bus[i,i].real)
                    J4[i, i] = Q_calc[i] / V[i] - (V[i] * Y_bus[i,i].imag)

        # --- D. Reduce Jacobian ---
        J1_red = J1[np.ix_(non_slack, non_slack)]
        J2_red = J2[np.ix_(non_slack, idx_pq)]
        J3_red = J3[np.ix_(idx_pq, non_slack)]
        J4_red = J4[np.ix_(idx_pq, idx_pq)]
        
        top_half = np.hstack((J1_red, J2_red))
        bot_half = np.hstack((J3_red, J4_red))
        J_final = np.vstack((top_half, bot_half))
        
        # --- DISPLAY JACOBIAN ---
        print("-" * 30)
        print(f"Jacobian Matrix (Iteration {it+1}):")
        with np.printoptions(precision=4, suppress=True, linewidth=100):
            print(J_final)
        print("-" * 30)

        # --- E. Prepare Mismatch Vector ---
        dP_red = dPa[non_slack]
        dQ_red = dQa[idx_pq]
        M_final = np.concatenate((dP_red, dQ_red))
        
        # --- F. Solve ---
        try:
            correction = np.linalg.solve(J_final, M_final)
        except np.linalg.LinAlgError:
            print("  Singular Matrix! System may be unstable.")
            break
            
        # --- G. Update State Variables ---
        n_ang = len(non_slack)
        dTheta = correction[:n_ang]
        dV = correction[n_ang:]
        
        for idx, val in enumerate(non_slack):
            Theta[val] += dTheta[idx]
        for idx, val in enumerate(idx_pq):
            V[val] += dV[idx]

    # --- Print Final Results ---
    print("\n" + "="*40)
    print("           FINAL LOAD FLOW RESULTS")
    print("="*40)
    print(f"{'Bus':<5} {'V (pu)':<10} {'Angle (deg)':<12} {'P (pu)':<10} {'Q (pu)':<10}")
    print("-" * 50)
    
    for i in range(num_buses):
        deg = np.degrees(Theta[i])
        print(f"{i+1:<5} {V[i]:<10.4f} {deg:<12.4f} {P_calc[i]:<10.4f} {Q_calc[i]:<10.4f}")