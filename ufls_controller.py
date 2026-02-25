class UFLS:
    def __init__(self):
        # Define the UFLS Stages:
        # We will use standard frequency thresholds for a 50Hz system.
        # "drop" is the percentage of the REMAINING load to shed.
        self.stages = [
            {"freq": 49.2, "drop": 0.05, "tripped": False, "name": "Stage 1 (5%)"},
            {"freq": 48.8, "drop": 0.10, "tripped": False, "name": "Stage 2 (10%)"},
            {"freq": 48.4, "drop": 0.15, "tripped": False, "name": "Stage 3 (15%)"}
        ]

    def check_and_shed(self, current_freq, bus_data):
        """
        Monitors frequency and sheds load if thresholds are crossed.
        Returns: (boolean indicating if a shed happened, list of alert messages)
        """
        shed_occurred = False
        alerts = []

        for stage in self.stages:
            # If frequency dips below threshold AND this stage hasn't fired yet
            if current_freq < stage["freq"] and not stage["tripped"]:
                
                # 1. Lockout this stage so it doesn't fire again
                stage["tripped"] = True
                shed_occurred = True
                
                # 2. Record the alert
                alerts.append(f"   [UFLS RELAY] {stage['name']} Tripped at {current_freq:.3f} Hz!")
                
                # 3. Physically reduce the load on the grid
                for b in bus_data:
                    if b['type'] == 3:  # Only shed load on PQ buses
                        
                        # Reduce Active Power (Pl) and Reactive Power (Ql)
                        b['Pl'] = b['Pl'] * (1.0 - stage["drop"])
                        b['Ql'] = b['Ql'] * (1.0 - stage["drop"])
                        
                        # Update the target specifications for the Newton-Raphson solver
                        b['P_spec'] = b['Pg'] - b['Pl']
                        b['Q_spec'] = b['Qg'] - b['Ql']

        return shed_occurred, alerts