class AGC:
    def __init__(self, K_p=0.5, K_i=0.2, target_freq=50.0):
        """
        Automatic Generation Control (Secondary Response)
        
        Args:
            K_p (float): Proportional Gain (Immediate correction)
            K_i (float): Integral Gain (Accumulated correction over time)
            target_freq (float): The grid setpoint (usually 50.0 Hz)
        """
        self.Kp = K_p
        self.Ki = K_i
        self.target = target_freq
        self.integral_error = 0.0  # The "memory" of the controller

    def calculate_regulation(self, current_freq, time_step):
        """
        Calculates the extra power needed to restore frequency.
        
        Returns:
            p_agc (float): Power adjustment in per-unit (pu)
        """
        # 1. Calculate Error (Positive if Freq < 50)
        error = self.target - current_freq
        
        # 2. Accumulate Integral (Area under the curve)
        self.integral_error += error * time_step
        
        # 3. PI Control Formula
        # P_out = (Kp * Error) + (Ki * Integral)
        p_agc = (self.Kp * error) + (self.Ki * self.integral_error)
        
        return p_agc

    def reset(self):
        """Clears the integral memory (useful for restarts)."""
        self.integral_error = 0.0