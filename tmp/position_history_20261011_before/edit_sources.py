from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def edit(name, replacements):
    path = ROOT / name
    raw = path.read_bytes()
    try:
        text = raw.decode('utf-8')
        encoding = 'utf-8'
    except UnicodeDecodeError:
        text = raw.decode('gbk')
        encoding = 'gbk'
    text = text.replace('\r\n', '\n')
    for old, new in replacements:
        if (new and new in text) or (not new and old not in text):
            continue  # Resume after a completed file; previous copies are retained.
        if text.count(old) != 1:
            raise ValueError(f'{name}: expected one occurrence: {old[:80]!r}')
        text = text.replace(old, new)
    path.write_bytes(text.replace('\n', '\r\n').encode(encoding))


edit('Algorithm/balance.c', [
    ('    c->x_m = limit(c->x_m + c->v_m_s * dt, LQR_X_LIMIT_M);',
     '    /* x is measured displacement, not an actuator integrator.\n'
     '     * Clamping x breaks x_dot=v and hides continued translation from LQR.\n'
     '     * Keep command saturation below; reset the origin on control reset. */\n'
     '    c->x_m += c->v_m_s * dt;'),
])
edit('Algorithm/balance.h', [
    ('#define LQR_X_LIMIT_M       0.3f\n', ''),
    ('    float x_m, v_m_s;',
     '    float x_m, v_m_s; /* Unclipped wheel odometry relative to control reset. */'),
])

edit('Applications/3508_CtrlApplications.h', [
    ('void WheelTorque_Send(FDCAN_HandleTypeDef *hfdcan, float T_left, float T_right);',
     '/* Diagnostic metadata only; feedback and command retain raw C620 signs. */\n'
     'typedef struct\n'
     '{\n'
     '    int16_t current_cmd_raw[2];\n'
     '    uint32_t tx_status, tx_cnt, tx_fail_cnt, last_cmd_tick;\n'
     '    uint32_t last_fb_tick[2], rx_cnt[2];\n'
     '} wheel_drive_diag_t;\n'
     'extern volatile wheel_drive_diag_t wheel_drive_diag;\n\n'
     'void WheelTorque_Send(FDCAN_HandleTypeDef *hfdcan, float T_left, float T_right);'),
])
edit('Applications/3508_CtrlApplications.c', [
    ('#include "3508_CtrlApplications.h"',
     '#include "3508_CtrlApplications.h"\n\nvolatile wheel_drive_diag_t wheel_drive_diag;'),
    ('    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);',
     '    wheel_drive_diag.current_cmd_raw[0] = cur[0];\n'
     '    wheel_drive_diag.current_cmd_raw[1] = cur[1];\n'
     '    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);\n'
     '    wheel_drive_diag.tx_status = (uint32_t)status;\n'
     '    if (status == HAL_OK)\n'
     '    {\n'
     '        wheel_drive_diag.tx_cnt++;\n'
     '        wheel_drive_diag.last_cmd_tick = HAL_GetTick();\n'
     '    }\n'
     '    else wheel_drive_diag.tx_fail_cnt++;'),
])
edit('Applications/DM_motor.c', [
    ('get_motor_measure(&motor_measure_L, RxData);',
     'get_motor_measure(&motor_measure_L, RxData);\n'
     '                    wheel_drive_diag.last_fb_tick[0] = HAL_GetTick();\n'
     '                    wheel_drive_diag.rx_cnt[0]++;'),
    ('get_motor_measure(&motor_measure_R, RxData);',
     'get_motor_measure(&motor_measure_R, RxData);\n'
     '                    wheel_drive_diag.last_fb_tick[1] = HAL_GetTick();\n'
     '                    wheel_drive_diag.rx_cnt[1]++;'),
])

edit('Tasks/Leg_Tasks.c', [
    ('    uint32_t hip_tx_status[4];\n    int16_t rpm_L, rpm_R, current_fb_L, current_fb_R;',
     '    uint32_t hip_tx_status[4], hip_fb_age_ms[4], hip_rx_cnt[4];\n'
     '    float joint_velocity_rad_s[4];\n'
     '    int16_t rpm_L, rpm_R, current_fb_L, current_fb_R;\n'
     '    int16_t wheel_current_cmd_raw[2];\n'
     '    uint32_t wheel_fb_age_ms[2], wheel_rx_cnt[2];\n'
     '    uint32_t wheel_tx_status, wheel_tx_cnt, wheel_tx_fail_cnt;\n'
     '    uint32_t can_tx_free[2], can_tx_error_count[2], can_rx_error_count[2];\n'
     '    uint32_t can_bus_off[2];'),
    ('volatile balance_diag_t balance_diag, balance_diag_standing, balance_diag_latest;',
     'volatile balance_diag_t balance_diag, balance_diag_standing, balance_diag_latest;\n'
     '/* 128 valid frames at 20 ms: retain roughly 2.56 s before feedback loss.\n'
     ' * next points to the next slot; newest is (next+127)%128. */\n'
     '#define BALANCE_HISTORY_COUNT 128U\n'
     'volatile balance_diag_t balance_diag_history[BALANCE_HISTORY_COUNT];\n'
     'volatile uint32_t balance_diag_history_next, balance_diag_history_count;'),
    ('    d->Tp_fb_L_Nm = DM8009_Ctrl[0].fb.torque + DM8009_Ctrl[1].fb.torque;\n'
     '    d->Tp_fb_R_Nm = -(DM8009_Ctrl[2].fb.torque + DM8009_Ctrl[3].fb.torque);\n', ''),
    ('    d->hip_enabled_mask = 0;\n    for (int i = 0; i < 4; ++i)',
     '    /* Command is current-cycle; feedback is the last asynchronous CAN sample.\n'
     '     * Record ages/counters and copy feedback atomically; never imply simultaneity. */\n'
     '    taskENTER_CRITICAL();\n'
     '    uint32_t feedback_now = HAL_GetTick();\n'
     '    d->hip_enabled_mask = 0;\n'
     '    for (int i = 0; i < 4; ++i)'),
    ('        d->hip_tx_status[i] = DM8009_Ctrl[i].last_tx_status;',
     '        d->hip_tx_status[i] = DM8009_Ctrl[i].last_tx_status;\n'
     '        d->hip_fb_age_ms[i] = feedback_now - DM8009_Ctrl[i].last_fb_tick;\n'
     '        d->hip_rx_cnt[i] = DM8009_Ctrl[i].rx_cnt;\n'
     '        d->joint_velocity_rad_s[i] = DM8009_Ctrl[i].fb.velocity;'),
    ('    d->current_fb_R = motor_measure_R.given_current;\n    d->valid = 1U;',
     '    d->current_fb_R = motor_measure_R.given_current;\n'
     '    d->Tp_fb_L_Nm = d->tau_fb_Nm[0] + d->tau_fb_Nm[1];\n'
     '    d->Tp_fb_R_Nm = -(d->tau_fb_Nm[2] + d->tau_fb_Nm[3]);\n'
     '    for (int i = 0; i < 2; ++i)\n'
     '    {\n'
     '        d->wheel_current_cmd_raw[i] = wheel_drive_diag.current_cmd_raw[i];\n'
     '        d->wheel_fb_age_ms[i] = feedback_now - wheel_drive_diag.last_fb_tick[i];\n'
     '        d->wheel_rx_cnt[i] = wheel_drive_diag.rx_cnt[i];\n'
     '    }\n'
     '    d->wheel_tx_status = wheel_drive_diag.tx_status;\n'
     '    d->wheel_tx_cnt = wheel_drive_diag.tx_cnt;\n'
     '    d->wheel_tx_fail_cnt = wheel_drive_diag.tx_fail_cnt;\n'
     '    taskEXIT_CRITICAL();\n'
     '    FDCAN_HandleTypeDef *buses[2] = {&hfdcan1, &hfdcan2};\n'
     '    for (int i = 0; i < 2; ++i)\n'
     '    {\n'
     '        FDCAN_ProtocolStatusTypeDef protocol = {0};\n'
     '        FDCAN_ErrorCountersTypeDef errors = {0};\n'
     '        (void)HAL_FDCAN_GetProtocolStatus(buses[i], &protocol);\n'
     '        (void)HAL_FDCAN_GetErrorCounters(buses[i], &errors);\n'
     '        d->can_tx_free[i] = HAL_FDCAN_GetTxFifoFreeLevel(buses[i]);\n'
     '        d->can_tx_error_count[i] = errors.TxErrorCnt;\n'
     '        d->can_rx_error_count[i] = errors.RxErrorCnt;\n'
     '        d->can_bus_off[i] = protocol.BusOff;\n'
     '    }\n'
     '    d->valid = 1U;'),
    ('    uint32_t latest_capture_tick = 0;',
     '    uint32_t latest_capture_tick = 0, history_capture_tick = 0;'),
    ('        if (ready && !balance_diag_standing.valid',
     '        if (ready && HAL_GetTick() - history_capture_tick >= 20U)\n'
     '        {\n'
     '            capture(&balance_diag_history[balance_diag_history_next], torque);\n'
     '            balance_diag_history_next = (balance_diag_history_next + 1U)\n'
     '                                       % BALANCE_HISTORY_COUNT;\n'
     '            if (balance_diag_history_count < BALANCE_HISTORY_COUNT)\n'
     '                balance_diag_history_count++;\n'
     '            history_capture_tick = HAL_GetTick();\n'
     '        }\n'
     '        if (ready && !balance_diag_standing.valid'),
])

edit('Matlab/tests/control_pair_test.c', [
    ('    /* Reproduce the screenshot failure: shortening/extension force saturates',
     '    /* Crossing the former +/-0.3 m clipping boundary must not hide travel.\n'
     '     * Replay symmetric forward/backward travel, then verify reset origin. */\n'
     '    for (int sign = -1; sign <= 1; sign += 2)\n'
     '    {\n'
     '        Balance_PairReset(&p, &left, &right);\n'
     '        float speed = sign * .72f;\n'
     '        float rl = speed / (WHEEL_RPM_TO_M_S * WHEEL_V_SIGN_L);\n'
     '        float rr = speed / (WHEEL_RPM_TO_M_S * WHEEL_V_SIGN_R);\n'
     '        for (int i = 0; i < 1000; ++i)\n'
     '            Balance_PairUpdate(&p, &left, &right, &vl, &vr, &ins, rl, rr, .001f);\n'
     '        CHECK(fabsf(p.common.x_m - speed) < .0001f);\n'
     '        CHECK(left.x_m == right.x_m);\n'
     '        CHECK(fabsf(left.T_Nm) <= LQR_T_LIMIT_NM\n'
     '              && fabsf(right.T_Nm) <= LQR_T_LIMIT_NM);\n'
     '        CHECK(fabsf(left.Tp_Nmm) <= LQR_TP_LIMIT_NMM);\n'
     '    }\n'
     '    Balance_PairReset(&p, &left, &right);\n'
     '    CHECK(p.common.x_m == 0 && left.x_m == 0 && right.x_m == 0);\n\n'
     '    /* Reproduce the screenshot failure: shortening/extension force saturates'),
])

print('Updated 7 source/test files; GBK and CRLF retained.')
