#!/usr/bin/env python3
"""Phase-1 geometry/load/ideal-beam screen for the shared flywheel concept.

This is a deterministic design screen, not hardware certification. Units are
SI in calculations and mm/N/MPa in the optional CalculiX beam model.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess

from run_global_frame_fea import ROOT, SOLVER, runtime_env

G = 9.80665
BALL_MASS = 0.0577
BALL_D = 0.066
PIVOT_X = 0.397
PIVOT_Z = 0.06430
NIP_U = 0.045
NIP_GAP = 0.058
BALL_NORMAL_K = 107309.0
MOTOR_KV_RPM_PER_V = 270.0
MOTOR_KT_NM_PER_A = 0.031
MOTOR_R_OHM = 0.039
BUS_V = 12.8


def point(theta_deg: float, u: float, w: float = 0.0) -> tuple[float, float]:
    t = math.radians(theta_deg)
    return (PIVOT_X + math.cos(t) * u - math.sin(t) * w,
            PIVOT_Z + math.sin(t) * u + math.cos(t) * w)


def annulus_volume(ro: float, ri: float, width: float) -> float:
    return math.pi * (ro * ro - ri * ri) * width


def printed_wheel_200() -> dict:
    """Primitive-volume estimate for an explicitly declared Ø200 printed trial wheel."""
    rho_petg, rho_tpu, rho_al = 1270.0, 1200.0, 2700.0
    rim_v = annulus_volume(.096, .084, .040)
    centre_v = annulus_volume(.032, .013, .040)
    spokes_v = 8 * .052 * .009 * .034 * .88
    tread_v = annulus_volume(.100, .096, .046)
    clamp_hub_v = annulus_volume(.023, .0041, .018)
    core_mass = (rim_v + centre_v + spokes_v) * rho_petg
    tread_mass = tread_v * rho_tpu
    metal_hub_mass = clamp_hub_v * rho_al
    wheel_mass = core_mass + tread_mass + metal_hub_mass
    # Ring/annulus inertias plus spoke estimate at their midpoint radius.
    rim_i = (rim_v*rho_petg) * (.096**2 + .084**2) / 2
    centre_i = (centre_v*rho_petg) * (.032**2 + .013**2) / 2
    spoke_i = (spokes_v*rho_petg) * ((.032+.084)/2)**2
    tread_i = (tread_v*rho_tpu) * (.100**2 + .096**2) / 2
    hub_i = metal_hub_mass * (.023**2 + .0041**2) / 2
    wheel_i = rim_i + centre_i + spoke_i + tread_i + hub_i
    return {
        "construction":"8-spoke PETG core, replaceable 4 mm radial TPU 95A tread, aluminium clamp hub",
        "assumed_densities_kg_m3":{"PETG":rho_petg,"TPU_95A":rho_tpu,"aluminium":rho_al},
        "estimated_mass_kg_each":wheel_mass,
        "primitive_polar_inertia_kg_m2_each":wheel_i,
        "inertia_bounds_kg_m2_each":[.5*wheel_mass*.1**2,wheel_mass*.1**2],
        "limit":"not weighed; slicer and assembled measurement required"
    }


def mass_model() -> dict:
    printed=printed_wheel_200()
    # Seller-listed pneumatic mass is deliberately kept separate from the
    # printed estimate; it has not been verified on the user's scale.
    pneumatic_wheel_mass=.900
    hub_mass=.026764
    pneumatic_rotating_mass=pneumatic_wheel_mass+hub_mass
    pneumatic_inertia=[.5*pneumatic_wheel_mass*.1**2,
                       pneumatic_wheel_mass*.1**2]

    upper_plate = .185 * .314 * .008 * 2700 * .94
    outboard_yokes = (2*.040*.080*.008 + 2*.040*.008*.043) * 2700
    motors = 2 * .49
    ancillary = .48
    common=upper_plate+outboard_yokes+motors+ancillary
    printed_module=2*printed["estimated_mass_kg_each"]+common
    pneumatic_module=2*pneumatic_rotating_mass+common
    return {
        "printed_d200":printed,
        "pneumatic_d200":{
            "seller_listed_tyre_mass_kg_each":pneumatic_wheel_mass,
            "mass_measurement_status":"UNMEASURED",
            "assumed_hub_mass_kg_each":hub_mass,
            "rotating_mass_kg_each":pneumatic_rotating_mass,
            "tyre_inertia_bounds_kg_m2_each":pneumatic_inertia,
            "inertia_basis":"0.5*m*R^2 .. m*R^2; hub inertia additional and small"
        },
        "module_mass_kg":{"printed_d200":printed_module,"pneumatic_d200":pneumatic_module},
        "structural_design_module_mass_kg":max(printed_module,pneumatic_module),
        "module_mass_breakdown_kg": {
            "upper_aluminium_plate": upper_plate, "outboard_aluminium_yokes": outboard_yokes,
            "two_D5065":motors,"trunnions_bearings_fasteners_gate_guide": ancillary
        }
    }


def variant_geometry(name: str, diameter_mm: float, width_mm: float, cad_clearance_band: list[float]) -> dict:
    radius=diameter_mm/2000
    half_width=width_mm/2000
    samples=[]
    for angle in range(-10,21):
        nip = point(angle, NIP_U)
        t = math.radians(angle)
        wheel_min=nip[1]-radius*abs(math.sin(t))-half_width*math.cos(t)
        front=nip[0]+radius*math.cos(t)+half_width*abs(math.sin(t))
        rear=nip[0]-radius*math.cos(t)-half_width*abs(math.sin(t))
        throat = point(angle, -.060)
        samples.append({"angle_deg": angle, "nip_z_mm": 1000*nip[1],
                        "wheel_min_z_mm": 1000*wheel_min,
                        "wheel_front_x_mm":1000*front,"wheel_rear_x_mm":1000*rear,
                        "rear_wheel_to_rail_end_x_clearance_mm":1000*(rear-.330),
                        "rear_throat_ball_center_z_mm": 1000*throat[1]})
    endpoint={}
    for key,a in (("intake_-10deg",-10),("launch_+20deg",20)):
        s=next(x for x in samples if x["angle_deg"]==a)
        endpoint[key]={"overall_length_mm":s["wheel_front_x_mm"]-(-165),
                       "overall_width_mm":677,"overall_height_mm":222,
                       "wheel_ground_clearance_mm":s["wheel_min_z_mm"],
                       "rear_wheel_to_rail_end_x_clearance_mm":s["rear_wheel_to_rail_end_x_clearance_mm"]}
    # Wheel cylinder axis is local W, so its lateral extent is centre +/- radius.
    outer=(diameter_mm+58)/2+diameter_mm/2
    return {
        "variant":name,"wheel_envelope_mm":[diameter_mm,width_mm],
        "wheel_centres_y_mm":[-(diameter_mm+58)/2,(diameter_mm+58)/2],
        "endpoint_dimensions":endpoint,
        "minimum_wheel_ground_clearance_mm":min(x["wheel_min_z_mm"] for x in samples),
        "minimum_rear_wheel_to_rail_end_x_clearance_mm":min(x["rear_wheel_to_rail_end_x_clearance_mm"] for x in samples),
        "flywheel_outer_y_mm":outer,
        "flywheel_to_drive_tyre_inner_face_lateral_clearance_mm":255.5-outer,
        "openscad_drive_clearance_bracket_mm":cad_clearance_band,
        "sweep":samples,
    }


def geometry_screen() -> dict:
    variants={
        "d200_main":variant_geometry("d200_main",200,50,[8.625,8.75]),
        "d160_comparison":variant_geometry("d160_comparison",160,44,[8.0,8.25]),
    }
    return {
        "angle_convention": "positive raises the front/local +U mouth",
        "pivot_mm": [1000*PIVOT_X, 0, 1000*PIVOT_Z],
        "nip_offset_from_pivot_mm": 1000*NIP_U,
        "endpoint_datums": {
            key: {"nip_mm": [1000*point(a,NIP_U)[0],0,1000*point(a,NIP_U)[1]],
                  "rear_throat_mm": [1000*point(a,-.060)[0],0,1000*point(a,-.060)[1]]}
            for key,a in (("intake_-10deg",-10),("launch_+20deg",20))
        },
        "variants":variants,
        "launch_rear_throat_ball_ground_clearance_mm":1000*point(20,-.060)[1]-BALL_D*500,
        "drive_measurement_basis":{
            "uncertainty_mm":"+/-2..3 tape measurement",
            "drive_axle_y_mm":"295..298; CAD nominal 297",
            "tyre_width_mm":83,"rail_outer_y_mm":245,"tyre_inner_y_mm":255.5,
            "rail_to_tyre_gap_mm":10.5,"overall_width_mm":677,
            "FIT0403_face_to_encoder_mm":"77..78 inward",
            "outer_hex_beyond_plate_mm":"47..49",
            "cable":"radial near encoder end, routed downward; no axial connector envelope"
        },
        "lateral_clearances_d200_mm":{
            "flywheel_outer_to_tyre_inner":26.5,"fixed_bearing_outer_to_tyre_inner":10.5,
            "moving_trunnion_outer_to_tyre_inner":8.5,
            "moving_trunnion_worst_with_measurement_uncertainty":5.5
        },
        "launch_port_contamination": {
            "ground_clearance_mm": 1000*point(20,-.060)[1] - BALL_D*500,
            "decision": "4.3 mm is unsuitable as an unsealed clay-court opening",
            "unapplied_alternative": "raise the launch-side dock or use an elevated internal feed shoe plus replaceable brush/labyrinth flap"
        },
        "side_cheek_variants_for_phase3": {
            "narrow_default_half_width_mm": [72,46],
            "wide_entry_half_width_mm": [110,46],
            "tradeoff": "narrow limits side strikes near the drive package; wide admits larger lateral error and relies more on counter-rotating-wheel self-centring"
        },
        "front_entry": "side cheeks only; no transverse lip or ramp ahead of first wheel contact",
        "full_lower_plate_result": "REJECTED: no full-length lower plate at W=-43 mm clears both endpoints",
        "accepted_connection":"forward/elevated supports at X=397, |Y|=237; no member passes through a flywheel",
        "bearing_arrangement_requirement":"self-aligning spherical bearings, or one locating and one floating bearing, because lifted-wheel frame twist must not preload both bearings",
        "main_xacro_observation_no_edit":{"wheel_y_m":.35,"geometric_wheel_separation_m":.70,
            "measured_real_track_m":"about 0.58..0.59","status":"finding only; Phase-1 CAD/bench alternative changed, xacro untouched"}
    }


def solve_ball_tread_split(total_closure: float, tread_stiffness_n_m: float | None) -> tuple[float,float,float]:
    """Solve delta_ball + 2*delta_tread = total closure."""
    if tread_stiffness_n_m is None:
        ball_delta = total_closure
    else:
        lo, hi = 0.0, total_closure
        for _ in range(80):
            ball_delta = (lo+hi)/2
            ball_force = BALL_NORMAL_K*ball_delta**1.5
            tread_force = tread_stiffness_n_m*(total_closure-ball_delta)/2
            if ball_force < tread_force:
                lo = ball_delta
            else:
                hi = ball_delta
        ball_delta = (lo+hi)/2
    tread_delta = (total_closure-ball_delta)/2
    return ball_delta,tread_delta,BALL_NORMAL_K*ball_delta**1.5


def traction_screen() -> dict:
    diametral_closure = BALL_D-NIP_GAP
    out = []
    compliance_cases = (
        ("rigid",None,"rigid tread; all 8 mm diametric closure assigned to ball","bound"),
        ("TPU_low",250000.0,"E=10 MPa, A=100 mm2, t=4 mm => k=250 N/mm/contact","realistic_order_screen"),
        ("TPU_nominal",1000000.0,"E=25 MPa, A=160 mm2, t=4 mm => k=1000 N/mm/contact","realistic_order_screen"),
        ("TPU_high",5000000.0,"E=50 MPa, A=400 mm2, t=4 mm => k=5000 N/mm/contact","realistic_order_screen"),
        ("legacy_foam_reference",3000.0,"k=3 N/mm/contact; reproduces the requested foam-like reference only","non_candidate_reference"),
        ("pneumatic_very_soft",10000.0,"unknown pressure/carcass; k=10 N/mm/contact","unknown_scan"),
        ("pneumatic_medium",30000.0,"unknown pressure/carcass; k=30 N/mm/contact","unknown_scan"),
        ("pneumatic_firm",100000.0,"unknown pressure/carcass; k=100 N/mm/contact","unknown_scan"),
    )
    for compliance,k_tread,note,family in compliance_cases:
        ball_delta,tread_delta,normal_per_wheel=solve_ball_tread_split(diametral_closure,k_tread)
        for diameter,contact_distance in ((200,.060),(160,.054)):
          for mu in (.3,.6,.9):
            tangential=2*mu*normal_per_wheel
            speed_gain=math.sqrt(2*tangential*contact_distance/BALL_MASS)
            out.append({"wheel_diameter_mm":diameter,"tread_compliance_case":compliance,
                        "case_family":family,
                        "tread_assumption":note,
                        "ball_tread_mu_assumed":mu,
                        "nominal_diametral_closure_mm":diametral_closure*1000,
                        "ball_compression_per_contact_mm":ball_delta*1000,
                        "tread_compression_per_contact_mm":tread_delta*1000,
                        "normal_force_n_per_wheel":normal_per_wheel,
                        "two_wheel_tangential_force_cap_n":tangential,
                        "ideal_constant_force_energy_screen_delta_v_mps":speed_gain,
                        "collection_0p8_mps_within_cap":speed_gain >= .8,
                        "guided_lift_collection_1p9_mps_within_cap":speed_gain >= 1.9,
                        "launch_12_mps_within_cap":speed_gain >= 12})
    motor_rpm=MOTOR_KV_RPM_PER_V*BUS_V
    surface_160=motor_rpm/60*2*math.pi*.080
    surface_200=motor_rpm/60*2*math.pi*.100
    def rpm_table(radius):
        return {str(v):v/(2*math.pi*radius)*60 for v in (.6,1.2,1.5,1.9,6,8,10,12,30)}
    return {
        "ball_tread_contact": out,
        "surface_speed_mps_to_rpm":{"d200":rpm_table(.1),"d160":rpm_table(.08)},
        "normal_force_basis": {
            "law":"F=107309*delta^1.5, SI, per ball contact",
            "nip_gap_mm":NIP_GAP*1000,
            "ball_diameter_mm":BALL_D*1000,
            "diametral_closure_mm":diametral_closure*1000,
            "equilibrium":"delta_ball + 2*delta_tread = 8 mm; F_ball(delta_ball)=k_tread*delta_tread for each tread",
            "warning":"TPU E/A contact area and pneumatic stiffness are estimates, not measurements"
        },
        "slip_and_motor_limit": {
            "root_cause_observation":"30 m/s circumference produced 8.25 m/s exit at mu=0.9 with traction saturation",
            "interpretation":"one saturated simulation datum only; 0.275 is explicitly not used as a predictive ratio",
            "D5065_no_load_rpm_at_12p8v":motor_rpm,
            "diameter_160_mm":{"surface_speed_mps":surface_160,
                               "rpm_required_for_30_mps":30/(2*math.pi*.080)*60,
                               "margin_to_30_mps_reference_mps":surface_160-30},
            "diameter_200_mm":{"surface_speed_mps":surface_200,
                               "rpm_required_for_30_mps":30/(2*math.pi*.100)*60,
                               "margin_to_30_mps_reference_mps":surface_200-30},
            "diameter_160_loss_vs_200_at_same_rpm":{"surface_speed_percent":100*(1-surface_160/surface_200),
                                        "surface_speed_mps":surface_200-surface_160},
            "warning":"exit speed requires dynamic traction validation; no zero-slip or empirical-ratio launch prediction is accepted"
        },
        "ground_contact_separate": {
            "statement": "wheel-ground friction is not reused as ball-tread friction",
            "screen_range": [0.5,0.7,0.9],
            "status": "reserved for drive/braking tests after phase-1 approval"
        },
        "direction_control": {
            "intake": "+Y wheel negative omega, -Y wheel positive omega; ball travels local -U; deep-guide search uses controlled 1.5..1.9 m/s surface speed",
            "launch": "both wheel signs reverse; ball travels local +U; zero-speed confirmation and closed isolation gate required before reversal"
        }
    }


def delivery_screen() -> dict:
    angle = math.radians(10)  # rearward intake exit is 10 degrees upward
    dx = .080
    trials = []
    for speed in (.6,.9,1.2):
        t = dx/(speed*math.cos(angle))
        dz = speed*math.sin(angle)*t - .5*G*t*t
        trials.append({"exit_speed_mps": speed, "flight_time_s": t,
                       "vertical_change_over_80mm_mm": 1000*dz})
    lift=.058
    ideal_lift_speed=math.sqrt(2*G*lift)
    half_efficiency_speed=math.sqrt(2*G*lift/.5)
    return {
        "unconstrained_ballistic_delivery": trials,
        "decision": "use an intake-only high contained guide; keep the basket front floor open",
        "reason": "80 mm free flight varies by about 68 mm vertically across the collection speed band; sqrt(2*g*dh) alone cannot establish capture",
        "high_guide": {
            "ball_center_path_mm": [[329,0,78],[305,0,82],[281,0,94],[257,0,111],
                                    [233,0,128],[209,0,136],[187,0,132],[163,0,132],[132,0,132]],
            "peak_lift_mm": 58,
            "ideal_minimum_entry_speed_mps": ideal_lift_speed,
            "minimum_with_50_percent_energy_transfer_mps": half_efficiency_speed,
            "controlled_surface_speed_search_mps": [1.5,1.7,1.9],
            "entry_tangent_deg": 9.5,
            "maximum_segment_slope_deg": 35.3,
            "function": "197 mm deep contained path, gradual rise and final 4 mm descent into the basket"
        },
        "guide_geometry": "short moving twin-rail handoff docks tangentially into a deep fixed roofed guide; tapered/shorter basket upper front creates the required curvature space",
        "retention": "basket has 141 mm wall height above its front floor, but rebound/escape is not validated until Gazebo/physical tests",
        "single_ball_feed": "fixed sloped pocket and reserved rotary singulator feed the low +20 degree port; detailed escapement remains open",
        "transition_interlock": [
            "close basket/feed isolation gate", "command both flywheels to zero",
            "verify speed below threshold", "rotate module at limited rate",
            "verify endpoint latch", "select opposite wheel direction", "open only the endpoint-appropriate gate"
        ]
    }


def cg_and_loads(mass: dict) -> dict:
    capacity_ball_count=28
    def solve(parts):
        total=sum(p[1] for p in parts)
        cg=[sum(p[1]*p[i] for p in parts)/total for i in (2,3,4)]
        rear_x,front_x=-.065,.300
        front_total=total*G*(cg[0]-rear_x)/(front_x-rear_x)
        rear_total=total*G-front_total
        return {"total_mass_kg":total,"cg_m":cg,
                "static_axle_loads_n":{"rear_total":rear_total,"front_total":front_total,
                                         "rear_each":rear_total/2,"front_each":front_total/2}}
    variants={}
    for name,module_mass in mass["module_mass_kg"].items():
        components_empty=[("bare_mobile_base",10.0,.100,0,.080),
            ("battery",5.59,-.055,0,.137),("shared_module",module_mass,.427,0,.105),
            ("basket_and_supports",1.50,.165,0,.125)]
        components_full=components_empty+[("28_tennis_balls",capacity_ball_count*BALL_MASS,.165,0,.125)]
        variants[name]={"empty_basket":solve(components_empty),"full_28_balls":solve(components_full)}
    return {
        "assumptions": {"bare_mobile_base_mass_kg":10.0,"bare_mobile_base_cg_m":[.100,0,.080],
                        "basket_support_mass_kg":1.50,"capacity_ball_count":capacity_ball_count,
                        "capacity_basis":"8.15 L gross CAD cavity; 24-28 practical estimate after guide and packing; 28 used for load screen",
                        "drive_axles_x_m":[-.065,.300]},
        "variants":variants,
        "battery_choice": "rear-centred upright placement x=-55 mm keeps y=0 and avoids the strong lateral imbalance of the prior side location",
        "battery_support": "two transverse tray rails and side restraints are packaging envelopes only; retention and local strength remain open"
    }


def pivot_loads(mass: dict, traction: dict) -> dict:
    module_mass=mass["structural_design_module_mass_kg"]
    # Conservative provisional CG local to pivot.
    cg_u,cg_w=.030,.025
    max_ball=max(x["two_wheel_tangential_force_cap_n"] for x in traction["ball_tread_contact"])
    cases=[]
    for angle in (-10,20):
        t=math.radians(angle)
        x_arm=math.cos(t)*cg_u-math.sin(t)*cg_w
        gravity_torque=module_mass*G*x_arm
        ball_torque=max_ball*NIP_U
        inertia_about_pivot=.115  # primitive estimate including two wheels/plate
        transition_torque=inertia_about_pivot*math.radians(30)
        hold=abs(gravity_torque)+ball_torque+transition_torque
        cases.append({"angle_deg":angle,"gravity_torque_nm":gravity_torque,
                      "mu_0p9_ball_reaction_torque_nm":ball_torque,
                      "30deg_s2_transition_torque_nm":transition_torque,
                      "combined_actuator_hold_screen_nm":hold})
    return {
        "module_cg_local_assumption_m":[cg_u,0,cg_w],
        "module_pitch_inertia_assumption_kg_m2":.115,
        "endpoint_cases":cases,
        "module_weight_n":module_mass*G,
        "bearing_vertical_design_load_n_each":module_mass*G,
        "basis":"2g vertical shared equally by two side bearings",
        "bearing_fore_aft_design_load_n_each":max_ball/2,
        "launch_ball_reaction_torque_nm":max_ball*NIP_U,
        "actuator_selection_load_nm":1.5*max(c["combined_actuator_hold_screen_nm"] for c in cases),
        "note":"old 9 N m and 0.413 N m Gamma values are neither capacity nor reused proof"
    }


def cad_collision_qa(work: Path) -> dict:
    angles=list(range(-10,21,2))
    # Recorded from the explicit OpenSCAD sweep command in P1-16. Invoking a
    # confined snap recursively from this solver script is not portable, so
    # CAD QA remains a separately reproducible geometry check.
    results=[{"angle_deg":angle,"hard_intersection":False} for angle in angles]
    return {
        "status":"NO_HARD_INTERSECTION_IN_SAMPLED_SWEEP",
        "sample_step_deg":2,
        "endpoint_angles_included":True,
        "angles":results,
        "minimum_clearance_brackets_mm":{"d200":[8.625,8.75],"d160":[8.0,8.25]},
        "method":"Minkowski-expand measured/assumed drive envelopes until first non-empty intersection",
        "other_empty_intersections":["moving-vs-frame","moving-vs-basket/battery","moving-vs-high-guide","wheel-vs-yoke"],
        "measurement_gate":"tape measurements carry +/-2..3 mm; verify with calipers and full cable bend radius before release",
        "reproduction":"openscad-nightly -D view=qa_drive_at_angle -D qa_angle=<angle> -o <output.stl> shared-flywheel-intake-launcher-phase1.scad"
    }


def local_arm_screen(pivot: dict) -> dict:
    length=.070; width=.045; thickness=.012; e=69e9
    vertical=pivot["bearing_vertical_design_load_n_each"]
    horizontal=pivot["bearing_fore_aft_design_load_n_each"]
    force=math.hypot(vertical,horizontal)
    inertia=width*thickness**3/12
    section_modulus=width*thickness**2/6
    moment=force*length
    return {"status":"SEPARATE_IDEAL_CANTILEVER_SCREEN_ONLY",
        "assumed_arm_mm":{"length":70,"width":45,"thickness":12,"material":"aluminium E=69 GPa"},
        "resultant_design_load_n":force,"root_moment_nm":moment,
        "nominal_bending_stress_mpa":moment/section_modulus/1e6,
        "tip_deflection_mm":force*length**3/(3*e*inertia)*1000,
        "not_covered":"bearing fit, bolt holes, weld/printed interfaces, bearing rating, fatigue and trunnion shoulder"}


def purchase_criteria(mass: dict, traction: dict) -> dict:
    no_load=MOTOR_KV_RPM_PER_V*BUS_V
    rpm_200=30/(2*math.pi*.1)*60
    rpm_160=30/(2*math.pi*.08)*60
    omega_200=rpm_200*2*math.pi/60
    current_limit=20.0
    voltage_at_200=rpm_200/MOTOR_KV_RPM_PER_V+current_limit*MOTOR_R_OHM
    def spinup(inertia_bounds,current):
        return [i*omega_200/(MOTOR_KT_NM_PER_A*current) for i in inertia_bounds]
    printed_i=mass["printed_d200"]["inertia_bounds_kg_m2_each"]
    pneumatic_i=mass["pneumatic_d200"]["tyre_inertia_bounds_kg_m2_each"]
    energy_printed=[2*.5*i*omega_200**2 for i in printed_i]
    energy_pneumatic=[2*.5*i*omega_200**2 for i in pneumatic_i]
    normal_cases={}
    for row in traction["ball_tread_contact"]:
        if row["wheel_diameter_mm"]==200 and row["ball_tread_mu_assumed"]==.9:
            normal_cases[row["tread_compliance_case"]]=row["normal_force_n_per_wheel"]
    return {
        "decision":"DO_NOT_SELECT_CONTROLLER_OR_RELEASE_PURCHASE_YET",
        "speed":{"D5065_no_load_rpm_at_12p8v":no_load,
            "d200_rpm_for_30_mps":rpm_200,"d200_fraction_of_no_load":rpm_200/no_load,
            "d160_rpm_for_30_mps":rpm_160,"d160_fraction_of_no_load":rpm_160/no_load,
            "d200_voltage_needed_at_20A_existing_model_v":voltage_at_200,
            "model":"rpm=Kv*(V-I*R), Kv=270 rpm/V, R=0.039 ohm; validate hot winding and loaded Kv"},
        "spinup_at_30mps_reference":{"printed_d200_seconds_at_8A":spinup(printed_i,8),
            "pneumatic_d200_seconds_at_8A":spinup(pneumatic_i,8),
            "acceptance":"reach command in <=12 s with robot stationary; no BMS trip; record current and temperature"},
        "droop_recovery":{"single_ball_peak_droop_percent_max":5,"recovery_s_max":1.0,
            "requirement":"closed-loop speed telemetry on both independent axes"},
        "battery_current":{"battery":"12.8 V 20 Ah, BMS >=20 A listed",
            "four_FIT0403_stall_screen_a":28,"two_D5065_provisional_20A_each_a":40,
            "finding":"either combination can exceed the BMS before auxiliaries",
            "acceptance":"measured total peak <=20 A and continuous <=16 A, or upgrade battery/BMS; interlock launch spin-up against traction/stall"},
        "low_speed_intake":{"d200_rpm_for_1p5_to_1p9_mps":[1.5/(2*math.pi*.1)*60,1.9/(2*math.pi*.1)*60],
            "d160_rpm_for_1p5_to_1p9_mps":[1.5/(2*math.pi*.08)*60,1.9/(2*math.pi*.08)*60],
            "corrected_normal_force_n_per_wheel_at_mu_scan_basis":normal_cases,
            "acceptance":"stable encoder-controlled rotation through ball entry at <=10 A/motor (0.31 Nm); no stall or rebound"},
        "braking":{"two_wheel_energy_j":{"printed_d200":energy_printed,"pneumatic_d200":energy_pneumatic},
            "average_absorption_w_for_5s_stop":{"printed_d200":[x/5 for x in energy_printed],"pneumatic_d200":[x/5 for x in energy_pneumatic]},
            "requirement":"controller must safely coast or manage regenerative bus rise with validated dump/brake path; direction reversal only below 50 rpm"},
        "encoder_and_controller_requirements":["one shaft encoder per D5065, AMT-102 class or equivalent, mechanically compatible second-shaft mounting",
            "resolution adequate at 140..230 rpm intake and ~2865 rpm launch; index and direction validation",
            "independent closed-loop current/speed control for both motors, encoder input and speed/current telemetry",
            "12.8 V operation, safe disable, current limits, bus over-voltage handling and braking/dump support",
            "controller selection deliberately deferred until bench data closes these gates"]}


def build_beam_model(braced: bool, mesh_mm: float):
    nodes, index, elements, sections = {}, {}, [], []
    def node(p):
        key=tuple(round(v,6) for v in p)
        if key not in index:
            index[key]=len(index)+1; nodes[index[key]]=key
        return index[key]
    def member(a,b,w=50,h=30,t=3.2,kind="BOX"):
        length=math.dist(a,b); count=max(1,math.ceil(length/mesh_mm))
        ids=[node(tuple(a[k]+(b[k]-a[k])*i/(2*count) for k in range(3))) for i in range(2*count+1)]
        eids=[]
        for i in range(count):
            eid=len(elements)+1; elements.append((eid,ids[2*i],ids[2*i+1],ids[2*i+2])); eids.append(eid)
        tangent=[(b[k]-a[k])/length for k in range(3)]
        axis=[0,0,1] if abs(tangent[2])<.95 else [0,1,0]
        dot=sum(axis[k]*tangent[k] for k in range(3)); normal=[axis[k]-dot*tangent[k] for k in range(3)]
        mag=math.sqrt(sum(v*v for v in normal)); normal=[v/mag for v in normal]
        sections.append((eids,w,h,t,kind,normal))
    stations=[-165,-65,35,200,208,240,272,300,320,330]
    for side in (-1,1):
        for a,b in zip(stations,stations[1:]): member((a,side*220,37),(b,side*220,37))
    member((-165,-220,37),(-165,220,37))
    if braced:
        ys=[-220,-160,0,160,220]
        for a,b in zip(ys,ys[1:]): member((240,a,37 if abs(a)==220 else 32),(240,b,37 if abs(b)==220 else 32),50,20)
        for side in (-1,1):
            member((208,side*220,37),(240,side*160,32),6,20,0,"RECT")
            member((272,side*220,37),(240,side*160,32),6,20,0,"RECT")
    bearings=[]
    for side in (-1,1):
        # Forward/elevated triangular support; the bearing is kept inboard of
        # the measured tyre inner face and no arm crosses a flywheel volume.
        member((300,side*220,37),(330,side*220,52),45,12,0,"RECT")
        member((330,side*220,52),(397,side*237,64.3),45,12,0,"RECT")
        member((320,side*220,37),(397,side*237,64.3),45,12,0,"RECT")
        bearings.append(node((397,side*237,64.3)))
    supports={
        "rear_right":node((-65,-220,37)),
        "rear_left":node((-65,220,37)),
        "front_right":node((300,-220,37)),
        "front_left":node((300,220,37)),
    }
    return nodes,elements,sections,supports,bearings


def support_constraints(supports: dict[str,int], lifted: str | None = None) -> list[tuple[int,int]]:
    # Four/three vertical tyre contacts plus one in-plane pin and one roller.
    # Rotations remain free; this removes rigid modes without clamping a beam node.
    constraints=[]
    for name,n in supports.items():
        if name != lifted:
            constraints.append((n,3))
    constraints += [(supports["rear_left"],1),(supports["rear_left"],2),
                    (supports["front_left"],2)]
    return constraints


def load_case(case_name: str, model, pivot: dict) -> tuple[list[tuple[int,int]],list[tuple[int,int,float]]]:
    nodes,_,_,supports,bearings=model
    constraints=support_constraints(supports,"front_right" if case_name=="one_wheel_lift" else None)
    loads=[]
    if case_name == "vertical_2g":
        for n in bearings: loads.append((n,3,-pivot["bearing_vertical_design_load_n_each"]))
    elif case_name == "launch_reaction":
        for n in bearings:
            loads += [(n,1,-pivot["bearing_fore_aft_design_load_n_each"]),
                      (n,5,pivot["launch_ball_reaction_torque_nm"]*1000/2)]
    elif case_name == "one_wheel_lift":
        for n in bearings: loads.append((n,3,-pivot["module_weight_n"]/2))
    elif case_name == "antisymmetric_racking":
        for n in bearings:
            side=1 if nodes[n][1]>0 else -1
            loads.append((n,1,side*pivot["bearing_fore_aft_design_load_n_each"]))
    else:
        raise ValueError(case_name)
    return constraints,loads


def write_deck(path, model, material, case_name, pivot):
    nodes,elements,sections,supports,bearings=model
    constraints,loads=load_case(case_name,model,pivot)
    lines=["*HEADING","Shared flywheel phase-1 ideal beam screen","*NODE,NSET=ALLN"]
    lines += [f"{n},"+",".join(map(str,p)) for n,p in nodes.items()]
    lines += ["*ELEMENT,TYPE=B32R,ELSET=ALLE"]+[",".join(map(str,e)) for e in elements]
    lines += ["*MATERIAL,NAME=POLYMER","*ELASTIC",f"{material['reference_elastic_modulus_gpa']*1000},{material['poisson_ratio']}"]
    for i,(ids,w,h,t,kind,normal) in enumerate(sections):
        lines += [f"*ELSET,ELSET=S{i}"]
        for j in range(0,len(ids),12): lines.append(",".join(map(str,ids[j:j+12])))
        lines += [f"*BEAM SECTION,ELSET=S{i},MATERIAL=POLYMER,SECTION={kind}",
                  f"{h},{w},{t},{t},{t},{t}" if kind=="BOX" else f"{h},{w}",
                  ",".join(map(str,normal))]
    lines += ["*NSET,NSET=BEARINGS",",".join(map(str,bearings)),"*BOUNDARY"]
    lines += [f"{n},{dof},{dof}" for n,dof in constraints]
    lines += ["*STEP","*STATIC","*CLOAD"]
    lines += [f"{n},{dof},{value}" for n,dof,value in loads]
    lines += ["*NODE PRINT,NSET=BEARINGS","U","*EL PRINT,ELSET=ALLE","S","*END STEP"]
    path.write_text("\n".join(lines)+"\n")


def parse_dat(path, bearings):
    displacement={}; peak=0.; mode=""
    for line in path.read_text().splitlines():
        low=line.lower()
        if "displacements" in low: mode="u"
        elif "stresses" in low: mode="s"
        fields=line.split()
        try:
            if mode=="u" and len(fields)==4 and int(fields[0]) in bearings:
                displacement[int(fields[0])]=[float(x) for x in fields[1:]]
            elif mode=="s" and len(fields)==8:
                sx,sy,sz,txy,txz,tyz=map(float,fields[2:])
                vm=math.sqrt(((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)/2+3*(txy*txy+txz*txz+tyz*tyz))
                peak=max(peak,vm)
        except ValueError: pass
    if set(displacement)!=set(bearings): raise RuntimeError("missing bearing displacement output")
    by_y=sorted(displacement.items(),key=lambda item:item[0])
    return {"max_bearing_displacement_mm":max(math.sqrt(sum(v*v for v in u)) for u in displacement.values()),
            "relative_bearing_x_mm":abs(by_y[0][1][0]-by_y[1][1][0]),
            "relative_bearing_z_mm":abs(by_y[0][1][2]-by_y[1][1][2]),
            "beam_integration_point_peak_von_mises_mpa":peak}


def fea_screen(work: Path, pivot: dict) -> dict:
    materials=json.loads((ROOT/"config/materials.json").read_text())["materials"]
    case_names=("vertical_2g","launch_reaction","one_wheel_lift","antisymmetric_racking")
    cases=[]
    for mesh in (20.,10.,5.):
        for braced in (False,True):
            model=build_beam_model(braced,mesh)
            for case_name in case_names:
                for name,material in materials.items():
                    stem=f"{case_name}_{'retained_brace' if braced else 'no_front_brace'}_{name.replace('-','_')}_{mesh:g}mm"
                    deck=work/f"{stem}.inp"; write_deck(deck,model,material,case_name,pivot)
                    run=subprocess.run([str(SOLVER),"-i",stem],cwd=work,env=runtime_env(),capture_output=True,text=True,timeout=60)
                    (work/f"{stem}.log").write_text(run.stdout+run.stderr)
                    if run.returncode or "*ERROR" in (run.stdout+run.stderr).upper(): raise RuntimeError(f"CalculiX failed: {stem}")
                    cases.append({"load_case":case_name,"front_structural_crossbar_retained":braced,
                                  "material":name,"mesh_mm":mesh,"element_count":len(model[1]),
                                  **parse_dat(work/f"{stem}.dat",model[-1])})
    finest=[]
    for case_name in case_names:
        for name in materials:
            no=next(c for c in cases if c["load_case"]==case_name and not c["front_structural_crossbar_retained"] and c["material"]==name and c["mesh_mm"]==5)
            yes=next(c for c in cases if c["load_case"]==case_name and c["front_structural_crossbar_retained"] and c["material"]==name and c["mesh_mm"]==5)
            metric="relative_bearing_x_mm" if case_name=="antisymmetric_racking" else "max_bearing_displacement_mm"
            base=no[metric]
            finest.append({"load_case":case_name,"material":name,"comparison_metric":metric,
                           "no_crossbar_response_mm":base,"x240_crossbar_response_mm":yes[metric],
                           "response_reduction_percent":100*(base-yes[metric])/base if base else None,
                           "x240_crossbar_peak_beam_stress_mpa":yes["beam_integration_point_peak_von_mises_mpa"]})
    convergence={}
    for case_name in case_names:
        changes=[]
        for braced in (False,True):
            for name in materials:
                fine=next(c for c in cases if c["load_case"]==case_name and c["front_structural_crossbar_retained"]==braced and c["material"]==name and c["mesh_mm"]==5)
                medium=next(c for c in cases if c["load_case"]==case_name and c["front_structural_crossbar_retained"]==braced and c["material"]==name and c["mesh_mm"]==10)
                changes.append(abs(fine["max_bearing_displacement_mm"]-medium["max_bearing_displacement_mm"])/fine["max_bearing_displacement_mm"])
        convergence[case_name]=max(changes)
    return {"status":"IDEAL_JOINT_ELASTIC_COMPARISON_NOT_PRINT_CERTIFICATION",
            "supersedes":"rear-X=-165 six-DOF clamp/cantilever results (12.9-19.8 mm)",
            "support_model":{
                "vertical":"U3 at X=-65 and X=300 drive mounts on both sides; front-right U3 released only for wheel-lift case",
                "in_plane":"rear-left U1/U2 pin plus front-left U2 roller to remove rigid modes",
                "rotations":"all support rotations free; no six-DOF clamp"
            },
            "load_cases":{
                "vertical_2g":"symmetric 2g module weight at the two trunnions",
                "launch_reaction":"fore-aft mu=0.9 rigid-tread reaction plus ball-force pitch moment at the trunnions",
                "one_wheel_lift":"1g module weight with front-right vertical support released",
                "antisymmetric_racking":"opposite fore-aft trunnion forces; crossbar assessed by relative bearing-X motion"
            },
            "retained_crossbar_x_mm":240,
            "maximum_relative_displacement_change_10_to_5mm_mesh":convergence,
            "finest_mesh_comparison":finest,
            "cases":cases,
            "limitations":["ideal bonded beam joints; bearing block, bolts and printed layer interfaces omitted",
                           "loads are phase-1 estimates, not measured dynamics",
                           "beam peak stress is not a hole/notch allowable or fatigue result",
                           "local arm screen is reported separately; bearing fit, bolts, trunnion shoulder and fatigue remain open"]}


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path,default=ROOT/"results/shared_flywheel_phase1/report.json")
    p.add_argument("--work",type=Path,default=ROOT/"fea/work/shared_flywheel_phase1")
    p.add_argument("--skip-fea",action="store_true")
    args=p.parse_args(); args.output.parent.mkdir(parents=True,exist_ok=True)
    mass=mass_model(); geometry=geometry_screen(); traction=traction_screen()
    pivot=pivot_loads(mass,traction)
    collision=cad_collision_qa(args.work)
    report={"status":"PHASE_1_REVIEWED_NOT_APPROVED_FOR_PHASE_2",
            "approval_gates":[
                "caliper-verify FIT0403, tyre, hub and radial cable bend envelopes; tape data are +/-2..3 mm",
                "choose and detail spherical or locating/floating pivot bearings and validate bolts/trunnions",
                "weigh the selected Ø200 wheel and measure its inertia, pneumatic stiffness and dynamic ball traction",
                "close total-current, braking-energy, encoder and controller gates before purchase",
                "obtain explicit approval before any xacro/Gazebo Phase-2 work"
            ],
            "mass":mass,"geometry":geometry,
            "traction":traction,"collection_delivery":delivery_screen(),
            "cg_and_wheel_loads":cg_and_loads(mass),"pivot_and_support_loads":pivot,
            "cad_collision_qa":collision,"local_bearing_arm_screen":local_arm_screen(pivot),
            "D5065_purchase_criteria":purchase_criteria(mass,traction)}
    if not args.skip_fea:
        args.work.mkdir(parents=True,exist_ok=True); report["ideal_beam_fea"]=fea_screen(args.work,pivot)
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


if __name__=="__main__": main()
