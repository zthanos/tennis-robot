#!/usr/bin/env python3
"""P2-85 quick strength check: ski R75/75 on X315 bolt + wall fork.

Plane XZ (mm, N). Per side = half of the ski load, except lateral handling,
which is put entirely on one bracket. Hand calculation, not FEA.
"""
from math import sin, cos, radians, pi, sqrt, asin

C = (383.0, 85.0); R = 75.0                       # arc centre / radius
B1 = (315.0, 10 + 75 * (1 - cos(asin(68 / 75))) + 11)   # X315 bolt
S = 14.0                                          # fork lock bolt spacing
T_LIP = radians(30.5); S_LIP = 7.6                # lip action point (wall mid)
TIP = (435.0, 10.0)

def arc(t, s=0.0):
    return (C[0] - (R - s) * sin(t), C[1] - (R - s) * cos(t))

def cross(r, f):            # moment about +Y for r=(x,z), f=(fx,fz)
    return r[1] * f[0] - r[0] * f[1]

P = arc(T_LIP, S_LIP); n = (sin(T_LIP), cos(T_LIP))
arm = abs(cross((P[0] - B1[0], P[1] - B1[1]), n))

# --- ball: 57 g, 4.5 m/s, 75 deg on R75 ---------------------------------
m, v = 0.057, 4.5
dp = 2 * m * v * sin(radians(37.5)); dt = (R / 1000 * radians(75)) / v
f_cent = m * v * v / (R / 1000)
print(f"ball: dp={dp:.3f} N.s over {dt*1000:.1f} ms -> mean {dp/dt:.1f} N,"
      f" centripetal {f_cent:.1f} N; design peak 50 N (any t on the arc)")

def ball_case(tb, F=50.0):
    d = (-sin(tb), -cos(tb))                # outward, line through centre
    f = (F * d[0], F * d[1])
    return f, cross((C[0] - B1[0], C[1] - B1[1]), f)

worst = max((abs(ball_case(radians(t))[1]), t) for t in range(0, 76))
print(f"ball 50 N worst moment about X315: {worst[0]/1000:.2f} N.m at t={worst[1]} deg"
      f" (resultant of the steady 15.4 N load acts at t=37.5 deg)")

cases = []
for tb in range(0, 76, 5):
    fb, mb = ball_case(radians(tb))
    for ang in range(0, 360, 15):           # 30 N handling at the ski tip
        fh = (30 * cos(radians(ang)), 30 * sin(radians(ang)))
        mh = cross((TIP[0] - B1[0], TIP[1] - B1[1]), fh)
        F = (fb[0] + fh[0], fb[1] + fh[1]); M = mb + mh
        cases.append((abs(M), F, M, tb, ang))
Mmax, F, M, tb, ang = max(cases)
Fside = (F[0] / 2, F[1] / 2); Mside = M / 2
N = abs(Mside) / arm                         # lip normal force per side
Ff = abs(Mside) / S                          # fork lock-bolt bearing force
Fb1 = sqrt((Fside[0] + Ff) ** 2 + Fside[1] ** 2) + N
print(f"\nworst 50+30 N: ball t={tb}, tip load {ang} deg -> ski M={M/1000:.2f} N.m,"
      f" F=({F[0]:.0f},{F[1]:.0f}) N")
print(f"per side: M={Mside/1000:.2f} N.m, lip arm {arm:.1f} mm -> N={N:.0f} N;"
      f" lock bolt {Ff:.0f} N; X315 bolt <= {Fb1:.0f} N (conservative sum)")

# --- friction (slot carries Z only by clamp) ----------------------------
pre, mu = 500.0, 0.25                         # retained PETG preload per M4
fric = 2 * mu * pre
print(f"slot friction per bolt {fric:.0f} N (2 faces, mu {mu}, {pre:.0f} N retained);"
      f" Z per side {abs(Fside[1]):.0f} N -> SF {2*fric/abs(Fside[1]):.0f}")

# --- bracket (printed on Y61 face: layers parallel to the web, XZ) ------
t_web, d = 8.0, 4.0
L = 20 + S + 4.5                              # slot length incl. ends
b_front = 12.0
for Fx, name in [(Fb1, "X315 bolt"), (Ff, "lock bolt")]:
    sig_lig = (Fx * L / 8) / (t_web * b_front ** 2 / 6)
    print(f"bracket {name}: bearing {Fx/(d*t_web):.1f} MPa,"
          f" front ligament (12 mm, fixed-fixed {L:.1f} mm) {sig_lig:.1f} MPa  [in-layer]")
w_neck = 26.6                                 # web width at Z56 (CAD)
M_neck = abs(Mside) + abs(Fside[0]) * (B1[1] + 10 - 54)
print(f"bracket web neck Z56: M={M_neck/1000:.2f} N.m, sigma"
      f" {M_neck/(t_web*w_neck**2/6):.1f} MPa  [in-layer]")

# Out-of-plane at the foot/web root (sigma_yy crosses the layers).
e = 65 - 48.5                                 # wall plane -> web mid-plane
Mx = abs(Fside[1]) * e + 30 * (B1[1] + 10 - 48)   # Fz eccentric + all lateral
parts = [  # (x_width, z0, z1) of the section in the plane Y=69
    (30, 42, 48),          # foot
    (30, 48, 50),          # R4 fillet (lower bound)
    (5, 48, 78),           # rear gusset at the web
    (5, 48, 56),           # front gusset
]
A = sum(w * (z1 - z0) for w, z0, z1 in parts)
zc = sum(w * (z1 - z0) * (z0 + z1) / 2 for w, z0, z1 in parts) / A
I = sum(w * (z1 - z0) ** 3 / 12 + w * (z1 - z0) * ((z0 + z1) / 2 - zc) ** 2
        for w, z0, z1 in parts)
c = max(abs(78 - zc), abs(zc - 42))
print(f"bracket root Y69 out-of-plane: Mx={Mx/1000:.2f} N.m, I={I:.0f} mm4,"
      f" sigma {Mx*c/I:.1f} MPa  [ACROSS layers]")
T = Mx / (75 - 61)
print(f"foot bolt Y75 tension {T:.0f} N; washer bearing {T/(pi/4*(9**2-4.5**2)):.1f} MPa")

# --- fork (printed on its outer face: layers parallel to XZ) ------------
t_f, b_arm = 12.7, 16.0
print(f"fork arm near hub: sigma {abs(Mside)/(t_f*b_arm**2/6):.1f} MPa  [in-layer]")
A_lip = 3.3 * 12.6                             # lower lip footprint
print(f"fork lip: transfer through layers tau {N/A_lip:.1f} MPa  [ACROSS layers]")
print(f"ski wall edge bearing: {N/(2.0*10):.1f} MPa (2 mm x ~10 mm)")
print("\nPETG references: in-layer UTS ~45-50 MPa, interlayer ~20-30 MPa;"
      " fatigue design target here: <=10 MPa in-layer, <=5 MPa across layers.")
