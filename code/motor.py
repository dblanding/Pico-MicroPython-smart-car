import time

class Motor():
    """
    Calculate and return PWM signal (uint 16) to acieve:
    1. Brief open loop operation while motor comes up to speed
    2. Closed loop operation using encoder feedback
        to maintain actual tick rate == target tick rate
    """

    def __init__(self, target_rate, multiplier, KP=-0.001, KD=1.2, KI=0):
        self.multiplier = multiplier
        self.target_rate = target_rate
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.initialized = False

    def update(self, tick_cnt):
        """
        Return pwm value to drive motor
        
        error = (actual tick rate - target tick rate)
        """

        if not self.initialized:
            # Initialize values on first update
            self.initialized = True
            self.prev_cnt = tick_cnt
            self.prev_rate = 0
            self.prev_time = time.ticks_ms()
            self.start_time = time.ticks_ms()
            self.rate_list = []
            pwm_val = 0
            print('rate Multiplier p_trim d_trim')
        else:
            # Calculate PWM value
            if (time.ticks_ms() - self.start_time) < 600:
                # Disable PID feedback while motor comes up to speed
                pwm_val = self.target_rate * self.multiplier
            else:
                # Enable PID feedback loop
                curr_time = time.ticks_ms()
                elapsed_time = curr_time - self.start_time
                delta_time = curr_time - self.prev_time
                delta_cnt = tick_cnt - self.prev_cnt
                
                # If driving backwards, speed is still positive
                delta_cnt = abs(delta_cnt)

                self.prev_time = curr_time
                self.prev_cnt = tick_cnt
                if delta_time == 0:
                    rate = 0
                else:
                    rate = (delta_cnt / delta_time) * 1_000
                proportional_error = rate - self.target_rate
                p_trim = self.KP * proportional_error

                if delta_time == 0:
                    derivative_error = 0
                else:
                    derivative_error = (rate - self.prev_rate) / delta_time
                    self.prev_rate = rate
                d_trim = self.KD * derivative_error

                integral_error = self._accumulated(proportional_error)
                i_trim = self.KI * integral_error

                self.multiplier += p_trim

                nominal = self.target_rate * self.multiplier
                pwm_val = int(nominal + p_trim + i_trim + d_trim)
                print(rate, self.multiplier, p_trim, d_trim)
                if pwm_val > 65_530:
                    pwm_val = 65_530

        return pwm_val

    def _accumulated(self, rate):
        """return sum of last 10 values of rate"""

        if len(self.rate_list) == 10:
            _ = self.rate_list.pop(0)
        self.rate_list.append(rate)
        return sum(self.rate_list)
