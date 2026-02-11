import numpy as np

def run_load_flow(Y_bus, bus_data, system_freq, max_iter=20, tol=1e-5, time_step=0):
    
    num_buses = len(bus_data)
    V = np.array([b['V'] for b in bus_data])
    Theta = np.array([b['theta'] for b in bus_data])
    P_spec = np.array([b['P_spec'] for b in bus_data])
    Q_spec = np.array([b['Q_spec'] for b in bus_data])
    types = np.array([b['type'] for b in bus_data]) 

    idx_slack = np.where(types == 1)[0]
    idx_pv    = np.where(types == 2)[0]
    idx_pq    = np.where(types == 3)[0]
    non_slack = np.concatenate((idx_pv, idx_pq))
    non_slack.sort()
    
    P_calc = np.zeros(num_buses)
    Q_calc = np.zeros(num_buses)

    for it in range(max_iter):
        P_calc.fill(0.0)
        Q_calc.fill(0.0)
        
        for i in range(num_buses):
            for k in range(num_buses):
                mag_Y = abs(Y_bus[i, k])
                ang_Y = np.angle(Y_bus[i, k])
                P_calc[i] += V[i] * V[k] * mag_Y * np.cos(Theta[i] - Theta[k] - ang_Y)
                Q_calc[i] += V[i] * V[k] * mag_Y * np.sin(Theta[i] - Theta[k] - ang_Y)

        dPa = P_spec - P_calc
        dQa = Q_spec - Q_calc
        
        mismatch = 0
        for i in non_slack: mismatch = max(mismatch, abs(dPa[i]))
        for i in idx_pq: mismatch = max(mismatch, abs(dQa[i]))
            
        if mismatch < tol:
            break 

        # Jacobian Construction
        J1 = np.zeros((num_buses, num_buses))
        J2 = np.zeros((num_buses, num_buses))
        J3 = np.zeros((num_buses, num_buses))
        J4 = np.zeros((num_buses, num_buses))
        
        for i in range(num_buses):
            for k in range(num_buses):
                mag_Y = abs(Y_bus[i, k])
                ang_Y = np.angle(Y_bus[i, k])
                if i != k:
                    term_a = V[i] * V[k] * mag_Y * np.sin(Theta[i] - Theta[k] - ang_Y)
                    term_b = V[i] * V[k] * mag_Y * np.cos(Theta[i] - Theta[k] - ang_Y)
                    J1[i, k] = term_a      
                    J2[i, k] = term_b / V[k]
                    J3[i, k] = -term_b
                    J4[i, k] = term_a / V[k] 
                else:
                    J1[i, i] = -Q_calc[i] - (V[i]**2 * Y_bus[i,i].imag)
                    J2[i, i] = P_calc[i] / V[i] + (V[i] * Y_bus[i,i].real)
                    J3[i, i] = P_calc[i] - (V[i]**2 * Y_bus[i,i].real)
                    J4[i, i] = Q_calc[i] / V[i] - (V[i] * Y_bus[i,i].imag)

        J1_red = J1[np.ix_(non_slack, non_slack)]
        J2_red = J2[np.ix_(non_slack, idx_pq)]
        J3_red = J3[np.ix_(idx_pq, non_slack)]
        J4_red = J4[np.ix_(idx_pq, idx_pq)]
        
        top = np.hstack((J1_red, J2_red))
        bot = np.hstack((J3_red, J4_red))
        J_final = np.vstack((top, bot))

        M_final = np.concatenate((dPa[non_slack], dQa[idx_pq]))
        
        try:
            correction = np.linalg.solve(J_final, M_final)
        except np.linalg.LinAlgError:
            return None, None, None, None

        n_ang = len(non_slack)
        dTheta = correction[:n_ang]
        dV = correction[n_ang:]
        for idx, val in enumerate(non_slack): Theta[val] += dTheta[idx]
        for idx, val in enumerate(idx_pq): V[val] += dV[idx]

    return V, Theta, P_calc, Q_calc