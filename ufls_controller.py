import csv
import os

class UFLS:
    def __init__(self, filename="ufls_training_data.csv"):
        # Define the UFLS Stages
        self.stages = [
            {"freq": 49.2, "drop": 0.05, "tripped": False, "name": "Stage 1 (5%)"},
            {"freq": 48.8, "drop": 0.10, "tripped": False, "name": "Stage 2 (10%)"},
            {"freq": 48.4, "drop": 0.15, "tripped": False, "name": "Stage 3 (15%)"}
        ]
        
        self.filename = filename
        
        # Create CSV and write header if it doesn't exist
        file_exists = os.path.isfile(self.filename)
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["Time_s", "Frequency_Hz", "RoCoF_Hz_s", "Total_Load_pu", "Action_Shed_Pct"])

    def check_and_shed(self, t, current_freq, rocof, bus_data):
        """
        Monitors frequency, sheds load, and logs data to CSV for AI training.
        """
        shed_occurred = False
        alerts = []
        action_taken_pct = 0.0  # AI Label: 0.0 means do nothing, >0 means shed

        for stage in self.stages:
            if current_freq < stage["freq"] and not stage["tripped"]:
                
                # Lockout this stage so it only fires once
                stage["tripped"] = True
                shed_occurred = True
                action_taken_pct += stage["drop"]
                alerts.append(f"   [UFLS RELAY] {stage['name']} Tripped at {current_freq:.3f} Hz!")
                
                # Physically reduce the load on PQ buses
                for b in bus_data:
                    if b['type'] == 3:  
                        b['Pl'] = b['Pl'] * (1.0 - stage["drop"])
                        b['Ql'] = b['Ql'] * (1.0 - stage["drop"])
                        
                        # Update the solver targets
                        b['P_spec'] = b['Pg'] - b['Pl']
                        b['Q_spec'] = b['Qg'] - b['Ql']

        # Calculate current load state for logging
        total_load = sum([b['Pl'] for b in bus_data])
        
        # Log real-time data to CSV for Machine Learning
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([t, round(current_freq, 4), round(rocof, 4), round(total_load, 4), round(action_taken_pct, 4)])

        return shed_occurred, alerts