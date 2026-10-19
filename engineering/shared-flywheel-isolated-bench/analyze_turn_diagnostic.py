#!/usr/bin/env python3
"""Evaluate the mass/contact and gyro turn diagnostic against frozen criteria."""
from __future__ import annotations

import argparse, csv, json, math
from pathlib import Path
import numpy as np

WHEELS = ("rear_left", "front_left", "rear_right", "front_right")
WINDOWS = {"spin_0p5": 4.0, "spin_1p0": 11.0, "moving_turn": 18.0}

def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path); p.add_argument("--criteria", type=Path, required=True)
    p.add_argument("--validation", type=Path, required=True); p.add_argument("--variant", required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    crit = json.loads(a.criteria.read_text())
    limits = crit["pass_criteria"]
    validation = json.loads(a.validation.read_text())
    rows = list(csv.DictReader(a.csv.open(newline="")))
    if len(rows) < 100: raise SystemExit(f"insufficient samples: {len(rows)}")
    names = rows[0].keys()
    data = {n: np.array([float(r[n]) for r in rows]) for n in names if n not in {"scenario", "case_name"}}
    scenarios = np.array([r["scenario"] for r in rows])
    case, mu = rows[0]["case_name"], float(rows[0]["mu"])
    mass = float(validation["total_mass_kg"]); cgx = float(validation["cg_at_zero_joint_pose_m"][0])
    xr, xf, half, radius, g = -0.065, 0.300, 0.297, 0.085, 9.80665
    wheelbase = xf-xr; dr, df = cgx-xr, xf-cgx
    nfront, nrear = mass*g*dr/wheelbase, mass*g*df/wheelbase
    required = 2*mu*mass*g*df*dr/wheelbase
    traction_available = mu*mass*g*half
    motor_available = 4*limits["torque_limit_nm"]*half/radius
    screen = {
      "mass_kg": mass, "cg_x_m": cgx, "front_normal_n": nfront, "rear_normal_n": nrear,
      "required_scrub_yaw_nm": required, "available_traction_yaw_nm": traction_available,
      "available_motor_yaw_nm": motor_available,
      "available_to_required_ratio": min(traction_available, motor_available)/required,
      "quasi_static_pass": min(traction_available, motor_available) >= required,
      "per_front_wheel_friction_torque_nm": mu*nfront*radius/2,
      "per_rear_wheel_friction_torque_nm": mu*nrear*radius/2
    }
    result = {"case": case, "mu": mu, "variant": a.variant, "sample_count": len(rows),
              "drift_reference_point": "base_link origin (X=0,Y=0), not CG and not ICR",
              "expected_load_centre_icr_body_m": [cgx, 0.0],
              "front_static_load_fraction": nfront/(nfront+nrear),
              "coulomb_screen": screen, "scenarios": {}, "loads": {}}
    settled = (data["time_s"] >= 1.5) & (data["time_s"] < 2.0)
    for scenario, start in WINDOWS.items():
        whole = scenarios == scenario; steady = whole & (data["time_s"] >= start)
        vcmd=float(np.median(data["target_v_m_s"][steady])); wcmd=float(np.median(data["target_w_rad_s"][steady]))
        vmean=float(np.mean(data["vx_body_m_s"][steady])); wmean=float(np.mean(data["wz_body_rad_s"][steady]))
        verr=abs(vmean-vcmd)/abs(vcmd) if abs(vcmd)>1e-9 else 0.; werr=abs(wmean-wcmd)/abs(wcmd)
        idx=int(np.flatnonzero(whole)[0]); drift=float(np.max(np.hypot(data["x_m"][whole]-data["x_m"][idx],data["y_m"][whole]-data["y_m"][idx])))
        # ICR in body coordinates follows v + omega x r = 0:
        # x_icr=-vy/w, y_icr=vx/w.  Separately track the world motion of the
        # expected load-centre ICR; this removes the base-origin circular chord.
        valid=steady & (np.abs(data["wz_body_rad_s"])>0.05)
        icr_x=-data["vy_body_m_s"][valid]/data["wz_body_rad_s"][valid]
        icr_y=data["vx_body_m_s"][valid]/data["wz_body_rad_s"][valid]
        yaw=data["yaw_rad"][whole]; cy=np.cos(yaw); sy=np.sin(yaw)
        px=data["x_m"][whole]+cy*cgx; py=data["y_m"][whole]+sy*cgx
        expected_icr_world_drift=float(np.max(np.hypot(px-px[0],py-py[0])))
        geometric_chord=float(np.max(np.hypot(cgx*(cy-cy[0]),cgx*(sy-sy[0]))))
        valid_idx=np.flatnonzero(valid); cyi=np.cos(data["yaw_rad"][valid]); syi=np.sin(data["yaw_rad"][valid])
        iw_x=data["x_m"][valid]+cyi*icr_x-syi*icr_y; iw_y=data["y_m"][valid]+syi*icr_x+cyi*icr_y
        iw_cx=float(np.median(iw_x)); iw_cy=float(np.median(iw_y))
        instantaneous_icr_world_p95=float(np.quantile(np.hypot(iw_x-iw_cx,iw_y-iw_cy),.95))
        instantaneous_icr_world_max=float(np.max(np.hypot(iw_x-iw_cx,iw_y-iw_cy)))
        ixm,iym=float(np.median(icr_x)),float(np.median(icr_y))
        ax0=math.cos(data["yaw_rad"][idx])*ixm-math.sin(data["yaw_rad"][idx])*iym
        ay0=math.sin(data["yaw_rad"][idx])*ixm+math.cos(data["yaw_rad"][idx])*iym
        actual_icr_geometric_chord=float(np.max(np.hypot(cy*ixm-sy*iym-ax0,sy*ixm+cy*iym-ay0)))
        torques=np.vstack([np.abs(data[f"{w}_command_nm"][steady]) for w in WHEELS])
        slips=np.vstack([data[f"{w}_slip_ratio"][steady] for w in WHEELS])
        caps=np.array([screen["per_rear_wheel_friction_torque_nm"],screen["per_front_wheel_friction_torque_nm"],screen["per_rear_wheel_friction_torque_nm"],screen["per_front_wheel_friction_torque_nm"]])[:,None]
        friction_over=float(np.mean(torques>caps)); saturation=float(np.mean(torques>=limits["saturation_threshold_fraction"]*limits["torque_limit_nm"]))
        rises=[]
        for w in WHEELS:
            z0=float(np.median(data[f"{w}_centre_z_m"][settled])); rises.append(float(np.max(data[f"{w}_centre_z_m"][whole]-z0)))
        roll=float(np.max(np.abs(np.degrees(data["roll_rad"][whole])))); pitch=float(np.max(np.abs(np.degrees(data["pitch_rad"][whole]))))
        checks=[werr<=limits["moving_yaw_rate_relative_error_max" if scenario=="moving_turn" else "spin_yaw_rate_relative_error_max"],
                roll<=limits["absolute_roll_deg_max"],pitch<=limits["absolute_pitch_deg_max"],max(rises)<=limits["max_wheel_centre_rise_m"],saturation<=limits["steady_torque_saturation_fraction_max"]]
        if scenario=="moving_turn": checks.append(verr<=limits["moving_linear_speed_relative_error_max"])
        else: checks.append(drift<=limits["spin_centre_translation_m_max"])
        result["scenarios"][scenario]={"target_v_m_s":vcmd,"target_w_rad_s":wcmd,"mean_v_m_s":vmean,"mean_w_rad_s":wmean,
          "linear_relative_error":verr,"yaw_relative_error":werr,"centre_translation_m":drift,"mean_slip":float(np.mean(slips)),"peak_slip":float(np.max(slips)),
          "instantaneous_icr_body_median_m":[float(np.median(icr_x)),float(np.median(icr_y))],
          "instantaneous_icr_body_p10_p90_m":{"x":[float(np.quantile(icr_x,.1)),float(np.quantile(icr_x,.9))],"y":[float(np.quantile(icr_y,.1)),float(np.quantile(icr_y,.9))]},
          "expected_load_centre_icr_world_drift_m":expected_icr_world_drift,
          "base_origin_geometric_chord_if_expected_icr_fixed_m":geometric_chord,
          "base_origin_geometric_chord_about_median_actual_icr_m":actual_icr_geometric_chord,
          "instantaneous_icr_world_p95_spread_m":instantaneous_icr_world_p95,
          "instantaneous_icr_world_max_spread_m":instantaneous_icr_world_max,
          "instantaneous_icr_x_offset_from_load_centre_m":float(ixm-cgx),
          "max_command_nm":float(np.max(torques)),"motor_saturation_fraction":saturation,"tyre_friction_limit_exceedance_fraction":friction_over,
          "max_roll_deg":roll,"max_pitch_deg":pitch,"max_wheel_centre_rise_m":max(rises),"pass":bool(all(checks))}
        result["loads"][scenario]={"tilt_peak_force_n":float(np.max(data["tilt_wrench_force_n"][whole])),
          "tilt_peak_torque_nm":float(np.max(data["tilt_wrench_torque_nm"][whole])),
          "tilt_p99_torque_nm":float(np.quantile(data["tilt_wrench_torque_nm"][whole],.99)),
          "tilt_position_mean_deg":float(np.degrees(np.mean(data["tilt_position_rad"][steady]))),
          "tilt_actuator_peak_command_nm":float(np.max(np.abs(data["tilt_command_nm"][whole]))),
          "tilt_actuator_p99_command_nm":float(np.quantile(np.abs(data["tilt_command_nm"][whole]),.99)),
          "peak_wheel_joint_force_n":float(max(np.max(data[f"{w}_wrench_force_n"][whole]) for w in WHEELS))}
    result["overall_pass"]=all(x["pass"] for x in result["scenarios"].values())
    a.output.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({"case":case,"mu":mu,"variant":a.variant,"pass":result["overall_pass"]}))
if __name__ == "__main__": main()
