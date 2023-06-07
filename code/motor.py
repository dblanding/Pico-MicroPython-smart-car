import time

class Motor():
    def __init__(self, target_rate, multiplier, KP=-0.0008, KI=0, KD=0):
        self.multiplier = multiplier
        self.target_rate = target_rate
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.initialized = False

    def update(self, tick_cnt):
        """return pwm value to drive motor"""

        if not self.initialized:
            # Initialize values on first update
            self.initialized = True
            self.prev_cnt = tick_cnt
            self.prev_rate = 0
            self.prev_time = time.ticks_ms()
            self.start_time = time.ticks_ms()
            self.rate_list = []
            pwm_val = 0
            print('rate Multiplier proportional_err integral_err')
        else:
            # Calculate PWM value
            if (time.ticks_ms() - self.start_time) < 400:
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
                d_trim = self.KD * derivative_error

                integral_error = self._accumulated(proportional_error)
                i_trim = self.KI * integral_error

                self.multiplier += p_trim

                nominal = self.target_rate * self.multiplier
                pwm_val = int(nominal + p_trim + i_trim + d_trim)
                print(rate, self.multiplier, proportional_error, integral_error)
                if pwm_val > 65_530:
                    pwm_val = 65_530

        return pwm_val

    def _accumulated(self, rate):
        """return average of last 10 values of rate"""

        if len(self.rate_list) == 10:
            _ = self.rate_list.pop(0)
        self.rate_list.append(rate)
        return sum(self.rate_list) / len(self.rate_list)
