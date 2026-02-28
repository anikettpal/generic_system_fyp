import random

class LoadFluctuator:
    def __init__(self, interval=5, min_inc=0.01, max_inc=0.05):
        self.interval = interval
        self.min_inc = min_inc
        self.max_inc = max_inc

    def fluctuate_load(self, t, bus_data):
        """
        Increases the load randomly by 1-5% on all PQ buses every 'interval' seconds.
        """
        # Trigger every 5 seconds, but skip t=0
        if t > 0 and t % self.interval == 0:
            # Generate a random percentage between 0.01 and 0.05
            increase_pct = random.uniform(self.min_inc, self.max_inc)
            
            for b in bus_data:
                if b['type'] == 3:  # Apply only to PQ (Load) buses
                    # Increase Active (Pl) and Reactive (Ql) power
                    b['Pl'] = b['Pl'] * (1.0 + increase_pct)
                    b['Ql'] = b['Ql'] * (1.0 + increase_pct)
                    
                    # Update the Newton-Raphson targets
                    b['P_spec'] = b['Pg'] - b['Pl']
                    b['Q_spec'] = b['Qg'] - b['Ql']
            
            alert = f"   [LOAD FLUCTUATION] Demand increased by {increase_pct*100:.2f}%!"
            return True, alert
            
        return False, ""