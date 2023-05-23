def move_forward():
    print('move forward')
    set_mtr_dirs('FWD', 'FWD')
    mtr_spd = int(target_tick_rate * 6)
    set_mtr_spds(mtr_spd, mtr_spd)
    start_time = time.ticks_ms()
    prev_time = start_time
    time.sleep(0.05)
    prev_enc_a = enc_a.value()
    prev_enc_b = enc_b.value()
    deadline = time.ticks_add(start_time, 3000)  # 3 seconds
    accum_err_a = []
    accum_err_b = []
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        curr_time = time.ticks_ms()
        curr_enc_a = enc_a.value()
        curr_enc_b = enc_b.value()
        delta_enc_a = curr_enc_a - prev_enc_a
        delta_enc_b = curr_enc_b - prev_enc_b
        delta_time = curr_time - prev_time
        prev_time = curr_time
        prev_enc_a = curr_enc_a
        prev_enc_b = curr_enc_b
        tick_rate_a = (delta_enc_a / delta_time) * 1000
        tick_rate_b = (delta_enc_b / delta_time) * 1000
        error_a = tick_rate_a - target_tick_rate
        error_b = tick_rate_b - target_tick_rate
        accum_err_a.append(error_a)
        accum_err_b.append(error_b)
        if len(accum_err_a) > 5:
            _ = accum_err_a.pop(0)
            integral_avg_a = sum(accum_err_a) / len(accum_err_a)
            i_trim_a = int(integral_avg_a * 15)
        else:
            i_trim_a = 0
        if len(accum_err_b) > 5:
            _ = accum_err_b.pop(0)
            integral_avg_b = sum(accum_err_b) / len(accum_err_b)
            i_trim_b = int(integral_avg_b * 15)
        else:
            i_trim_b = 0
        p_trim_a = int(error_a * 5)
        p_trim_b = int(error_b * 5)
        set_mtr_spds(mtr_spd - p_trim_a - i_trim_a,
                     mtr_spd - p_trim_b - i_trim_b)
        print(tick_rate_a, tick_rate_b, p_trim_a, p_trim_b, i_trim_a, i_trim_b)
        time.sleep(0.1)
        # await asyncio.sleep(0.1)
