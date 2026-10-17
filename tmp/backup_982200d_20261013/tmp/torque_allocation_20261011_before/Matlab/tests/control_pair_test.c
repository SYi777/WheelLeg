#include "balance.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned checks;
#define CHECK(expr) do { ++checks; if (!(expr)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #expr); exit(1); } } while (0)

static float angle_difference(float a, float b)
{
    float d = a - b;
    if (d > 3.14159265f) d -= 6.2831853f;
    if (d < -3.14159265f) d += 6.2831853f;
    return d;
}

typedef struct { double x, y; } reference_point;

/* Independent circle-intersection FK, double precision, libm trig.
 * The production FK uses a half-angle solver and CMSIS table interpolation.
 * Differencing its interpolation knots is not the exact geometric derivative. */
static reference_point reference_fk(double qa, double qb)
{
    double ax = L1 * cos(qa), ay = L1 * sin(qa);
    double cx = L2 * cos(qb), cy = L2 * sin(qb);
    double dx = cx - ax, dy = cy - ay, d = sqrt(dx * dx + dy * dy);
    double along = (L3 * L3 - L4 * L4 + d * d) / (2 * d);
    double height = sqrt(L3 * L3 - along * along);
    double cross_o = dx * (-ay) - dy * (-ax);
    double branch = cross_o > 0 ? -1 : 1; /* elbow down: opposite side to O */
    reference_point b = {ax + along * dx / d - branch * height * dy / d,
                         ay + along * dy / d + branch * height * dx / d};
    return b;
}

int main(void)
{
    balance_pair_t p = {0};
    balance_t left = {0}, right = {0};
    vmc_t vl = {0}, vr = {0};
    INS_t ins = {0};
    vl.theta_rel_body_rad = vr.theta_rel_body_rad = THETA_E_RAD / LQR_LEG_SIGN;
    Balance_PairUpdate(&p, &left, &right, &vl, &vr, &ins, 0, 0, .001f);
    CHECK(fabsf(left.T_Nm) < 1e-5f && fabsf(right.T_Nm) < 1e-5f);
    CHECK(fabsf(left.Tp_Nmm) < 1e-3f);

    /* Nose down, with WORLD leg angle held at equilibrium. */
    ins.Pitch = -.02f / LQR_PITCH_SIGN;
    vl.theta_rel_body_rad = vr.theta_rel_body_rad = (THETA_E_RAD - .02f) / LQR_LEG_SIGN;
    Balance_PairUpdate(&p, &left, &right, &vl, &vr, &ins, 0, 0, .001f);
    CHECK(p.T_common_model_Nm > 0 && p.Tp_common_model_Nm > 0);
    CHECK(fabsf(left.T_Nm - right.T_Nm) < 1e-6f);

    Balance_PairReset(&p, &left, &right);
    ins.Pitch = 0;
    vl.theta_rel_body_rad = vr.theta_rel_body_rad = THETA_E_RAD / LQR_LEG_SIGN;
    /* Pure wheel differential: mean motion is zero; correction must remove energy. */
    float rpm_l = .2f / (WHEEL_RPM_TO_M_S * WHEEL_V_SIGN_L);
    float rpm_r = -.2f / (WHEEL_RPM_TO_M_S * WHEEL_V_SIGN_R);
    Balance_PairUpdate(&p, &left, &right, &vl, &vr, &ins, rpm_l, rpm_r, .001f);
    CHECK(fabsf(p.common.v_m_s) < 1e-6f && fabsf(p.common.x_m) < 1e-6f);
    CHECK(fabsf(p.T_common_model_Nm) < 1e-5f);
    CHECK((left.T_Nm / LQR_WHEEL_SIGN) * p.v_L_m_s
          + (right.T_Nm / LQR_WHEEL_SIGN) * p.v_R_m_s < 0);

    /* A leg split does not get promoted to wheel steering by longitudinal K. */
    Balance_PairReset(&p, &left, &right);
    vl.theta_rel_body_rad += .04f;
    vr.theta_rel_body_rad -= .04f;
    Balance_PairUpdate(&p, &left, &right, &vl, &vr, &ins, 0, 0, .001f);
    CHECK(fabsf(left.T_Nm) < 1e-5f && fabsf(right.T_Nm) < 1e-5f);

    /* Symmetric motion uses exactly one shared position integral. */
    Balance_PairReset(&p, &left, &right);
    vl.theta_rel_body_rad = vr.theta_rel_body_rad = THETA_E_RAD / LQR_LEG_SIGN;
    for (int i = 0; i < 100; ++i)
        Balance_PairUpdate(&p, &left, &right, &vl, &vr, &ins, rpm_l, -rpm_r, .001f);
    CHECK(fabsf(p.common.x_m - .02f) < 1e-5f);
    CHECK(left.x_m == right.x_m);
    CHECK(left.T_Nm == right.T_Nm);

    /* Independent finite differences validate endpoint velocity/Jacobian. */
    unsigned poses = 0;
    for (int i = 0; i < 8; ++i)
    {
        float qa = .25f + .06f * i, qb = 2.9f - .04f * i;
        float dqa = .7f, dqb = -.2f, eps = .002f;
        FKResult a = fivebar_fk(qa - eps * dqa, qb - eps * dqb);
        FKResult b = fivebar_fk(qa + eps * dqa, qb + eps * dqb);
        if (a.status != FK_OK || b.status != FK_OK) continue;
        VMC_StateUpdateWithVelocity(&vl, qa, qb, dqa, dqb, .001f);
        CHECK(vl.fk_s.fk.status == FK_OK && vl.fk_s.jac_ok);
        reference_point ra = reference_fk(qa - eps * dqa, qb - eps * dqb);
        reference_point rb = reference_fk(qa + eps * dqa, qb + eps * dqb);
        reference_point rc = reference_fk(qa, qb);
        CHECK(fabs(vl.fk_s.fk.B.x - rc.x) < .05 && fabs(vl.fk_s.fk.B.y - rc.y) < .05);
        float La = (float)sqrt(ra.x * ra.x + ra.y * ra.y);
        float Lb = (float)sqrt(rb.x * rb.x + rb.y * rb.y);
        float axis_a = (float)atan2(ra.y, ra.x), axis_b = (float)atan2(rb.y, rb.x);
        CHECK(fabsf(vl.L_t_dot_mm_s - (Lb - La) / (2 * eps)) < .12f);
        CHECK(fabsf(vl.theta_rel_body_dot_rad_s
                  - angle_difference(axis_b, axis_a) / (2 * eps)) < .001f);
        float Fx, Fy, ta, tb;
        VMC_force_polar_to_cart(&vl.fk_s.fk.B, 70.0f, 1500.0f, &Fx, &Fy);
        fivebar_force(&vl.fk_s.J, Fx, Fy, &ta, &tb);
        CHECK(fabsf(ta * dqa + tb * dqb
                    - (70 * vl.L_t_dot_mm_s + 1500 * vl.theta_rel_body_dot_rad_s)) < .1f);
        ++poses;
    }
    CHECK(poses >= 6);
    printf("PASS: %u checks; %u FK poses; actual control and CMSIS code, compiled for Cortex-M7.\n", checks, poses);
    return 0;
}
