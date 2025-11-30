import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# -------------------------------
# Mini Cheetah parameters (from your headers)
# -------------------------------
body_length = 0.19 * 2
body_width  = 0.049 * 2
abad_link_length = 0.062
hip_link_length  = 0.209
knee_link_length = 0.195
knee_link_y_offset = 0.004

# Rotor locations from MiniCheetah.h
# - Ab/Ad rotor: relative to BODY center
# - Hip rotor:   relative to Ab/Ad joint
# - Knee rotor:  relative to HIP (zero -> hip origin)
abad_rotor_location = np.array([0.125, 0.049, 0.0])  # body-centered offset
hip_rotor_location  = np.array([0.0, 0.04, 0.0])     # from Ab/Ad outward
knee_rotor_location = np.array([0.0, 0.0, 0.0])      # at HIP frame origin

# -------------------------------
# Helpers
# -------------------------------
def with_leg_signs(v, leg_id):
    """Apply withLegSigns convention exactly like the C++."""
    x, y, z = v
    if leg_id == 0:   return np.array([ x, -y,  z])  # FL
    if leg_id == 1:   return np.array([ x,  y,  z])  # FR
    if leg_id == 2:   return np.array([-x, -y,  z])  # HL
    if leg_id == 3:   return np.array([-x,  y,  z])  # HR
    raise ValueError("Invalid leg id")

# Body/Joint relative placements (translations only; orientations not drawn)
abad_location = np.array([body_length, body_width, 0.0]) * 0.5  # Ab/Ad at body corners
hip_location  = np.array([0.0, abad_link_length, 0.0])          # OUTWARD from Ab/Ad
knee_location = np.array([0.0, 0.0, -hip_link_length])          # DOWN from Hip

# Body rectangle for visual reference
body_corners = np.array([
    [ body_length/2,  body_width/2, 0.0],
    [ body_length/2, -body_width/2, 0.0],
    [-body_length/2, -body_width/2, 0.0],
    [-body_length/2,  body_width/2, 0.0],
    [ body_length/2,  body_width/2, 0.0],
])

# -------------------------------
# Compute joint & rotor positions
# -------------------------------
legs_data, side_sign = [], -1
for leg_id in range(4):
    # Joints
    p_abad = with_leg_signs(abad_location, leg_id)
    p_hip  = p_abad + with_leg_signs(hip_location, leg_id)   # NOTE: translation not rotated
    p_knee = p_hip + knee_location
    y_off  = knee_link_y_offset if side_sign < 0 else -knee_link_y_offset
    p_foot = p_knee + np.array([0.0, y_off, -knee_link_length])

    # Rotors (per code: knee rotor is mounted on HIP, zero offset)
    p_abad_rotor = with_leg_signs(abad_rotor_location, leg_id)         # body-based
    p_hip_rotor  = p_abad + with_leg_signs(hip_rotor_location, leg_id) # Ab/Ad-based
    p_knee_rotor = p_hip + knee_rotor_location                         # HIP-based

    legs_data.append({
        "abad": p_abad, "hip": p_hip, "knee": p_knee, "foot": p_foot,
        "abad_rotor": p_abad_rotor, "hip_rotor": p_hip_rotor, "knee_rotor": p_knee_rotor
    })
    side_sign *= -1

# -------------------------------
# Plot
# -------------------------------
fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection='3d')

# Body outline
ax.plot(body_corners[:,0], body_corners[:,1], body_corners[:,2], linewidth=2)

colors = ["r", "g", "b", "m"]
names  = ["FL", "FR", "HL", "HR"]

for d, c, name in zip(legs_data, colors, names):
    # Links
    xs = [d["abad"][0], d["hip"][0], d["knee"][0], d["foot"][0]]
    ys = [d["abad"][1], d["hip"][1], d["knee"][1], d["foot"][1]]
    zs = [d["abad"][2], d["hip"][2], d["knee"][2], d["foot"][2]]
    ax.plot(xs, ys, zs, color=c, linewidth=2)
    ax.scatter(xs, ys, zs, color=c, s=40)

    # Joint labels
    ax.text(*d["abad"], f"{name}: Ab/Ad", fontsize=9, color=c)
    ax.text(*d["hip"],  f"{name}: Hip",   fontsize=9, color=c)
    ax.text(*d["knee"], f"{name}: Knee",  fontsize=9, color=c)
    ax.text(*d["foot"], f"{name}: Foot",  fontsize=9, color=c)

    # Rotor markers + labels
    ax.scatter(*d["abad_rotor"], marker="^", s=70)
    ax.text(*d["abad_rotor"], f"{name}: Ab/Ad rotor", fontsize=8)

    ax.scatter(*d["hip_rotor"], marker="s", s=70)
    ax.text(*d["hip_rotor"], f"{name}: Hip rotor", fontsize=8)

    ax.scatter(*d["knee_rotor"], marker="D", s=70)
    ax.text(*d["knee_rotor"], f"{name}: Knee rotor (at hip)", fontsize=8)

# Legend for rotor markers
legend_elements = [
    Line2D([0], [0], marker='^', linestyle='None', markersize=8, label='Ab/Ad rotor'),
    Line2D([0], [0], marker='s', linestyle='None', markersize=8, label='Hip rotor'),
    Line2D([0], [0], marker='D', linestyle='None', markersize=8, label='Knee rotor (hip-mounted)'),
]
ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0.02, 0.98))

# Axes, view, aspect
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Mini Cheetah: Joints, Links, and Rotor Locations (knee rotor at hip)')
ax.view_init(elev=18, azim=55)
ax.set_box_aspect([1, body_width/body_length, 0.6])
plt.tight_layout()
plt.show()
