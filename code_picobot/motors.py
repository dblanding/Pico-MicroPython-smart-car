import time

class Motors():
    """
    Calculate and return PWM signals for both motors using encoder feedback
    in a PIDC controller:
        
    P - Proportional
    I - Integral
    D - Derivative
    C - Count difference (a/b)

    """

    def __init__(self, target_tick_rate, fwd=True, multiplier=11,
                 EKP=-0.002, EKI=-0.001, EKD=-0.05, EKC=-0.01):
        self.multiplier = multiplier
        self.ttr = target_tick_rate
        self.fwd = fwd
        self.rate_list_a = []
        self.rate_list_b = []
        # encoder control loop coefficients
        self.EKP = EKP  # proportional
        self.EKI = EKI  # integral
        self.EKD = EKD  # derivative
        self.EKC = EKC  # count difference
        self.initialized = False

    def update(self, curr_cnt_a, curr_cnt_b):
        """
        return (pwm_a, pwm_b)
        
        error = (actual tick rate - target tick rate)
        """

        if not self.initialized:
            # Initialize values on first update
            self.initialized = True
            self.start_cnt_a = curr_cnt_a
            self.prev_cnt_a = curr_cnt_a
            self.start_cnt_b = curr_cnt_b
            self.prev_cnt_b = curr_cnt_b
            self.prev_rate_a = 0
            self.prev_rate_b = 0
            self.prev_time = time.ticks_ms()
            self.start_time = time.ticks_ms()
            pwm_a = 0
            pwm_b = 0
            print('p_trim_a, i_trim_a, d_trim_a, c_trim_a')
            # print('total_cnt_a, total_cnt_b, rate_a, rate_b')
        else:
            # control motor speed on subsequent updates
            
            # calculate delta tick counts since previous update
            curr_time = time.ticks_ms()
            delta_time = curr_time - self.prev_time
            delta_cnt_a = curr_cnt_a - self.prev_cnt_a
            delta_cnt_b = curr_cnt_b - self.prev_cnt_b
            
            # Keep track of accumulated total counts
            total_cnt_a = curr_cnt_a - self.start_cnt_a
            total_cnt_b = curr_cnt_b - self.start_cnt_b
            
            # speed is always positive
            delta_cnt_a = abs(delta_cnt_a)
            delta_cnt_b = abs(delta_cnt_b)

            # calculate current tick rate (ticks per sec)
            self.prev_time = curr_time
            self.prev_cnt_a = curr_cnt_a
            self.prev_cnt_b = curr_cnt_b
            if delta_time == 0:
                # avoid dividing by zero
                rate_a = 0
                rate_b = 0
            else:
                # delta_time is in ms thus * 1_000 below
                rate_a = (delta_cnt_a / delta_time) * 1_000
                rate_b = (delta_cnt_b / delta_time) * 1_000

            # proportional
            # error = actual tick rate - target tick rate
            p_err_a = rate_a - self.ttr
            p_trim_a = p_err_a * self.EKP
            p_err_b = rate_b - self.ttr
            p_trim_b = p_err_b * self.EKP
            
            # integral
            i_err_a = self._accumulated(p_err_a, self.rate_list_a)
            i_trim_a = self.EKI * i_err_a
            i_err_b = self._accumulated(p_err_b, self.rate_list_b)
            i_trim_b = self.EKI * i_err_b


            # derivative
            if delta_time == 0:
                d_err_a = 0
                d_err_b = 0
            else:
                d_err_a = (rate_a - self.prev_rate_a) / delta_time
                d_err_b = (rate_b - self.prev_rate_b) / delta_time
                self.prev_rate_a = rate_a
                self.prev_rate_b = rate_b
            d_trim_a = d_err_a * self.EKD
            d_trim_b = d_err_b * self.EKD
            
            # yaw caused by total_cnt_a total_cnt_b difference
            cnt_err = total_cnt_a - total_cnt_b
            
            # need to reverse for driving backwards
            if not self.fwd:
                cnt_err = -cnt_err
            
            c_trim_a = cnt_err * self.EKC
            c_trim_b = -cnt_err * self.EKC

            print(p_trim_a, i_trim_a, d_trim_a, c_trim_a)
            # print(total_cnt_a, total_cnt_b, rate_a, rate_b)

            mult_a = self.multiplier + p_trim_a + i_trim_a + d_trim_a + c_trim_a
            mult_b = self.multiplier + p_trim_b + i_trim_b + d_trim_b + c_trim_b

            # calculate PWM values
            pwm_a = int(self.ttr * (mult_a))
            pwm_b = int(self.ttr * (mult_b))
            if pwm_a > 65_530:
                pwm_a = 65_530
            if pwm_b > 65_530:
                pwm_b = 65_530
            
        return pwm_a, pwm_b

    def _accumulated(self, rate, rate_list):
        """return sum of previous values of rate"""

        if len(rate_list) == 100:
            _ = rate_list.pop(0)
        rate_list.append(rate)
        return sum(rate_list)
