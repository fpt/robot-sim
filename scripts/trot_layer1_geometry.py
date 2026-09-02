#!/usr/bin/env python
"""Layer 1 of docs/reflex_quad_12dof_trot_plan.md: geometry, statics and
mobility -- no GPU, no change to the actual robot model yet.

    .venv/bin/python scripts/trot_layer1_geometry.py

This is the layer the plan says the morphology decision happens at ("2 yaw
axes, not 1 central axis" was answered here in minutes) and where a servo
go/no-go should come out *before* buying twelve STS3215s.  Every number below
reads from the CURRENT config/*.yaml (the running 8-DOF robot) and treats the
would-be 12-DOF additions (2 roll pivots, 2 yaw joints, front/rear frame
split) as a candidate on paper only.  Nothing here touches robot.yaml,
asset_builder.py or JOINT_NAMES: per docs/reflex_quad_12dof_trot_plan.md's
own layering, mock.py's physics loop is hard-coded to 4 legs x 2 joints
(q[0::2]/q[1::2], 2*i/2*i+1 indexing) and would need real extension work
(lateral translation, yaw, a support-diagonal inverted-pendulum mode) before
the model itself can change without breaking every existing experiment.

Trust rule from the plan (section "シミュレーション結論の信頼度ルール"):
believe the geometry, the statics, the mobility numbers and the *comparisons*
between candidates; do not believe any of this script's absolute numbers
(theta_max, servo curve, contact friction) are the real hardware's -- they
are placeholders until docs/reflex_quad_sts3215_isaac_eval_plan.md section
5.3's bench measurements land in servo.yaml.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from reflex_quad.robot import LegGeometry  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = ROOT / "logs" / "trot_layer1"
GRAVITY = 9.81


def _cfg(name: str) -> dict:
    return yaml.safe_load((ROOT / "config" / name).read_text())


# ===========================================================================
# Part 1 -- mobility per morphology candidate x support-foot combination
# ===========================================================================
#
# Chebyshev-Grubler-Kutzbach, planar (lambda = 3, all joints here are
# revolute about the vertical/z axis so f=1 each):
#
#   M = 3 * (n_links - 1) - sum(3 - f_i)   over all joints i
#
# A foot in firm no-slip contact is modelled as a revolute joint at a fixed
# point: it fixes the foot's (x, y) but lets it pivot (yaw) freely, which is
# the right idealisation for asking "can this support state propel the body
# by turning joints, without any foot sliding" -- exactly the plan's
# question. Verified against the two numbers the plan already worked out by
# hand: 2 yaw axes + spine on diagonal support -> M=1, one central yaw axis
# -> M=0 (both reproduced by SEGMENTS below before this script does anything
# else, so a wrong model here would show up immediately).
def planar_mobility(n_links: int, joint_dofs: list[int]) -> int:
    return 3 * (n_links - 1) - sum(3 - f for f in joint_dofs)


SEGMENTS = {
    "8dof (current, one rigid body)": {
        "segments": ["Body"],
        "yaw_joints": [],
        "foot_segment": {"FL": "Body", "FR": "Body", "RL": "Body", "RR": "Body"},
    },
    "12dof, 2 yaw axes (chosen)": {
        "segments": ["Front", "Spine", "Rear"],
        "yaw_joints": [("Front", "Spine"), ("Spine", "Rear")],
        "foot_segment": {"FL": "Front", "FR": "Front", "RL": "Rear", "RR": "Rear"},
    },
    "12dof, 1 central yaw axis (rejected)": {
        "segments": ["Front", "Rear"],
        "yaw_joints": [("Front", "Rear")],
        "foot_segment": {"FL": "Front", "FR": "Front", "RL": "Rear", "RR": "Rear"},
    },
}

SUPPORT_STATES = {
    "diagonal FR+RL": ["FR", "RL"],
    "diagonal FL+RR": ["FL", "RR"],
    "front pair FL+FR": ["FL", "FR"],
    "rear pair RL+RR": ["RL", "RR"],
    "tripod FL+FR+RL": ["FL", "FR", "RL"],
    "all four": ["FL", "FR", "RL", "RR"],
}


def mobility(candidate: str, support_feet: list[str]) -> int:
    c = SEGMENTS[candidate]
    n_links = 1 + len(c["segments"])  # +1 for ground
    joint_dofs = [1] * len(c["yaw_joints"])
    joint_dofs += [1] * len(support_feet)
    return planar_mobility(n_links, joint_dofs)


def print_mobility_table() -> None:
    print("=" * 78)
    print("Part 1 -- mobility, planar Grubler, per candidate x support state")
    print("=" * 78)
    header = f"{'candidate':<38}" + "".join(f"{s:<18}" for s in SUPPORT_STATES)
    print(header)
    for candidate in SEGMENTS:
        row = f"{candidate:<38}"
        for feet in SUPPORT_STATES.values():
            row += f"{mobility(candidate, feet):<18}"
        print(row)
    print("""
M > 0: the support state can be driven (turn a joint, body/legs move) without
       any stance foot sliding.  M = 0: locked, needs slip to move at all.
       M < 0: over-constrained (statically indeterminate) by that amount.

Caveat this table cannot hide: plain Grubler mobility is a net count over the
*whole* mechanism.  For front-pair / rear-pair / tripod support here, two
feet pin the SAME rigid segment, which alone locks that segment to the
ground with one redundant constraint (M=-1 in the 8dof row shows this in
isolation) while leaving the unsupported segment free to swing on its own
yaw joints -- the M shown for those rows is real, but it is the unsupported
segment swinging in the air, not a propulsive DOF shared across loaded feet.
Diagonal support is the only state in this table where the counted DOF is
unambiguously shared between two independently-grounded segments -- which is
exactly why the plan picked a diagonal trot, not front/rear-pair or tripod.
""")


# ===========================================================================
# Part 2 -- yaw anti-phase closed-loop kinematics: stride estimate
# ===========================================================================
#
# docs/reflex_quad_12dof_trot_plan.md's own formula: stride per half-cycle =
# foot's lateral offset from the yaw axis x sin(yaw range).  hip_y is that
# lateral offset (the yaw axis is assumed to run through the frame's
# centreline, under the spine) with a foot in firm no-slip contact tracing
# an arc as its frame yaws.
def yaw_stride(robot_cfg: dict, yaw_range_deg: float = 30.0) -> dict:
    hip_y = abs(float(robot_cfg["legs"][0]["hip_y"]))  # symmetric, any leg does
    yaw_range = np.radians(yaw_range_deg)
    stride_half_cycle = hip_y * np.sin(yaw_range)
    return {
        "hip_y_m": hip_y,
        "yaw_range_deg": yaw_range_deg,
        "stride_half_cycle_m": stride_half_cycle,
        "stride_per_cycle_m": 2 * stride_half_cycle,
    }


def print_yaw_stride(result: dict) -> None:
    print("=" * 78)
    print("Part 2 -- yaw-driven stride (legs held still, propulsion from yaw only)")
    print("=" * 78)
    print(f"  hip lateral offset       {result['hip_y_m'] * 1000:6.1f} mm")
    print(f"  yaw range                +-{result['yaw_range_deg']:.0f} deg")
    print(f"  stride / half-cycle      {result['stride_half_cycle_m'] * 1000:6.1f} mm")
    print(f"  stride / full cycle      {result['stride_per_cycle_m'] * 1000:6.1f} mm")
    # docs/reflex_quad_sts3215_isaac_eval_plan.md section 4: gait frequency
    # 2-3 Hz is the yaw reaction-cancellation oscillation band, reused here
    # as the candidate stepping cadence.
    for f_hz in (2.0, 3.0):
        half_cycle_s = 1.0 / (2 * f_hz)
        speed = result["stride_half_cycle_m"] / half_cycle_s
        print(f"  at {f_hz:.0f} Hz gait freq        "
              f"{speed * 1000:6.0f} mm/s forward, from yaw alone")
    print()


# ===========================================================================
# Part 3 -- tip-over time-constant map, servo torque/speed overlay
# ===========================================================================
#
# Inverted-pendulum recovery window, exactly the plan's formula:
#   tau = sqrt(h / g)
#   t_avail = tau * ln(2 * theta_max / eps0)
# theta_max (recoverable-tilt envelope) is explicitly "the central quantity
# to measure on mock" in the plan -- not known yet.  Swept here, not assumed.
def t_avail(h: float, theta_max_rad: float, eps0_rad: float) -> float:
    tau = np.sqrt(h / GRAVITY)
    return tau * np.log(2 * theta_max_rad / eps0_rad)


def swing_inertia(robot_cfg: dict, servo_cfg: dict) -> float:
    """Leg rotational inertia about the hip, point mass at each link's own
    centre (upper link's COM at l1/2, lower link's COM at l1 + l2/2), plus
    the servo's own rotor inertia already reflected to the joint
    (servo.yaml: joint.rotor_inertia).  A deliberately simple estimate --
    the real number is bench-measured territory (sts3215 plan section 5.3),
    this only needs to be right to a factor for a go/no-go contour."""
    leg = robot_cfg["leg"]
    l1, l2 = leg["upper_length"], leg["lower_length"]
    m1, m2 = leg["upper_mass"], leg["lower_mass"]
    i_leg = m1 * (l1 / 2) ** 2 + m2 * (l1 + l2 / 2) ** 2
    return i_leg + servo_cfg["joint"]["rotor_inertia"]


def required_leg_swing_angle(geom: LegGeometry, leg_swing_stride_m: float) -> float:
    """Hip angle sweep to move the foot leg_swing_stride_m forward, split
    evenly fore/aft of the nominal stance, via the exact IK (not a small-
    angle approximation)."""
    h = geom.nominal_height
    hip_lead, _ = geom.ik(h, forward=leg_swing_stride_m / 2)
    hip_trail, _ = geom.ik(h, forward=-leg_swing_stride_m / 2)
    return abs(hip_lead - hip_trail)


def tip_over_map(robot_cfg: dict, servo_cfg: dict, geom: LegGeometry) -> None:
    print("=" * 78)
    print("Part 3 -- tip-over recovery window vs. required swing speed/torque")
    print("=" * 78)

    tau_max = float(servo_cfg["limits"]["tau_max"])
    qd_max = float(servo_cfg["limits"]["qd_max"])
    i_swing = swing_inertia(robot_cfg, servo_cfg)
    eps0 = np.radians(0.8)  # the plan's own worked example

    # Split a representative total stride between yaw's contribution (Part 2,
    # 30 deg range) and what the legs must still cover themselves.
    stride_targets_mm = [60.0, 100.0, 140.0]
    yaw = yaw_stride(robot_cfg)
    print(f"  swing inertia about hip   {i_swing * 1e6:6.1f} g*cm^2 "
          f"(leg + rotor, see swing_inertia() for the approximation)")
    print(f"  yaw contribution          {yaw['stride_half_cycle_m'] * 1000:6.1f} mm / half-cycle")
    print()
    print(f"  {'total stride':>14}  {'leg must cover':>16}  {'d(hip)':>8}")
    leg_swing_by_target = {}
    for target_mm in stride_targets_mm:
        leg_mm = max(0.0, target_mm - yaw["stride_half_cycle_m"] * 1000)
        dtheta = required_leg_swing_angle(geom, leg_mm / 1000)
        leg_swing_by_target[target_mm] = dtheta
        print(f"  {target_mm:>11.0f} mm  {leg_mm:>13.0f} mm  {np.degrees(dtheta):>6.1f} deg")
    print()

    # Headline scenario for the contour: the *largest* stride target, and
    # theta_max swept down close to eps0 -- the middle target (100 mm) was
    # GO everywhere in this servo's own operating range (current config has
    # a 30-80x torque margin there), which is a real result but not a useful
    # map; pushing to where the boundary actually is says more about how
    # much headroom exists before this servo becomes the limit.
    dtheta = leg_swing_by_target[stride_targets_mm[-1]]
    h_range = np.linspace(robot_cfg["stance"]["min_height"],
                           robot_cfg["stance"]["max_height"], 60)
    theta_max_range_deg = np.linspace(1.0, 20, 60)
    margin = np.zeros((len(theta_max_range_deg), len(h_range)))  # >0 = GO

    for j, h in enumerate(h_range):
        for i, theta_max_deg in enumerate(theta_max_range_deg):
            ta = t_avail(h, np.radians(theta_max_deg), eps0)
            if ta <= 0 or dtheta <= 0:
                margin[i, j] = -1.0
                continue
            omega_peak = 2 * dtheta / ta               # triangular velocity profile
            alpha = 4 * dtheta / ta ** 2
            tau_req = i_swing * alpha
            if omega_peak >= qd_max:
                margin[i, j] = -1.0                     # speed alone infeasible
                continue
            tau_avail = tau_max * (1 - omega_peak / qd_max)  # linear torque/speed line
            margin[i, j] = tau_avail - tau_req

    fig, ax = plt.subplots(figsize=(7, 5))
    cs = ax.contourf(h_range * 1000, theta_max_range_deg, margin,
                      levels=[-1, 0, margin.max() if margin.max() > 0 else 1],
                      colors=["#d9534f", "#5cb85c"], alpha=0.6)
    ax.contour(h_range * 1000, theta_max_range_deg, margin, levels=[0],
               colors="black", linewidths=1.5)
    ax.set_xlabel("body height h [mm]")
    ax.set_ylabel("theta_max, recoverable tilt envelope [deg]")
    ax.set_title(
        f"servo go/no-go: {stride_targets_mm[-1]:.0f} mm stride "
        f"({np.degrees(dtheta):.1f} deg hip sweep from legs), eps0=0.8 deg\n"
        "green = servo torque/speed covers the swing; red = it does not"
    )
    ax.axvline(robot_cfg["stance"]["height"] * 1000, color="k", ls="--", lw=0.8,
               label=f"current nominal height ({robot_cfg['stance']['height'] * 1000:.0f} mm)")
    ax.legend(loc="upper right", fontsize=8)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "tip_over_servo_map.png"
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"  contour saved: {out_path.relative_to(ROOT)}")

    # A couple of headline numbers at the current config's own height.
    h0 = robot_cfg["stance"]["height"]
    print(f"\n  at the current nominal height ({h0 * 1000:.0f} mm), "
          f"{stride_targets_mm[-1]:.0f} mm stride, eps0=0.8 deg:")
    for theta_max_deg in (1, 2, 4, 8, 16):
        ta = t_avail(h0, np.radians(theta_max_deg), eps0)
        omega_peak = 2 * dtheta / ta
        alpha = 4 * dtheta / ta ** 2
        tau_req = i_swing * alpha
        tau_avail = tau_max * (1 - omega_peak / qd_max) if omega_peak < qd_max else float("-inf")
        go = "GO" if (omega_peak < qd_max and tau_req <= tau_avail) else "NO-GO"
        print(f"    theta_max={theta_max_deg:>2.0f} deg  t_avail={ta * 1000:5.0f} ms  "
              f"omega_peak={omega_peak:5.2f} rad/s  tau_req={tau_req:6.4f} Nm  "
              f"tau_avail={tau_avail:6.4f} Nm  -> {go}")
    print()


# ===========================================================================
# Part 4 -- roll-correctable disturbance cone
# ===========================================================================
#
# docs/reflex_quad_12dof_trot_plan.md: roll only corrects the LATERAL
# component of a tip about the support diagonal; the longitudinal component
# is a swing-time problem (Part 3).  Modelled as a single-axis inverted
# pendulum: the roll actuator's torque must exceed gravity's overturning
# torque at the tip angle, m*g*sin(angle)*h_com, or the joint's own
# mechanical range wins first.  tau_roll_max is the STS3215's own spec from
# docs/reflex_quad_sts3215_isaac_eval_plan.md section 1 (19 kgf.cm @ 7.4V);
# roll_range_deg=15 is that same document's assumed working range
# ("phi <= 15 deg" keeps the leg-plane gravity error under cos(phi) >= 0.97).
def roll_disturbance_cone(robot_cfg: dict, tau_roll_max: float = 1.9,
                           roll_range_deg: float = 15.0) -> dict:
    m_total = float(robot_cfg["body"]["mass"]) + 4 * (
        float(robot_cfg["leg"]["upper_mass"]) + float(robot_cfg["leg"]["lower_mass"])
    )
    h_com = float(robot_cfg["stance"]["height"]) + float(robot_cfg["body"]["height"]) / 2
    torque_limited_rad = np.arcsin(
        np.clip(tau_roll_max / (m_total * GRAVITY * h_com), -1.0, 1.0)
    )
    torque_limited_deg = np.degrees(torque_limited_rad)
    binding = "mechanical range" if torque_limited_deg > roll_range_deg else "servo torque"
    return {
        "m_total_kg": m_total,
        "h_com_m": h_com,
        "torque_limited_deg": torque_limited_deg,
        "roll_range_deg": roll_range_deg,
        "recoverable_deg": min(torque_limited_deg, roll_range_deg),
        "binding_constraint": binding,
    }


def print_roll_cone(result: dict) -> None:
    print("=" * 78)
    print("Part 4 -- roll-correctable disturbance cone (lateral tip only)")
    print("=" * 78)
    print(f"  total mass                {result['m_total_kg']:.3f} kg")
    print(f"  CoM height (approx)       {result['h_com_m'] * 1000:.0f} mm")
    print(f"  torque-limited recovery   {result['torque_limited_deg']:.1f} deg")
    print(f"  mechanical roll range     +-{result['roll_range_deg']:.0f} deg")
    print(f"  binding constraint        {result['binding_constraint']}")
    print(f"  ==> recoverable lateral tilt: {result['recoverable_deg']:.1f} deg\n")


def main() -> None:
    robot_cfg = _cfg("robot.yaml")
    servo_cfg = _cfg("servo.yaml")
    geom = LegGeometry(robot_cfg)

    print_mobility_table()
    print_yaw_stride(yaw_stride(robot_cfg))
    tip_over_map(robot_cfg, servo_cfg, geom)
    print_roll_cone(roll_disturbance_cone(robot_cfg))

    print("=" * 78)
    print("Trust rule (docs/reflex_quad_12dof_trot_plan.md): the mobility "
          "numbers, the comparison between candidates, and the geometry are "
          "load-bearing.  theta_max, the servo torque/speed line and the "
          "swing inertia are placeholders -- recalibrate from "
          "docs/reflex_quad_sts3215_isaac_eval_plan.md section 5.3's bench "
          "measurements before trusting the GO/NO-GO calls above.")


if __name__ == "__main__":
    main()
