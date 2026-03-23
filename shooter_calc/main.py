import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

#90 roter rps absolute max
#93.75 flywheel rps absolute max
#4 in (0.1016m) diameter flywheel
v0_max_limit = 29.9236 # m/s v_0 absolute max
v0_min_limit = 0.0

#43 <= theta <= 87
theta_min_limit = 43.0
theta_max_limit = 87.0

hopper_height = 1.8288 #meters
robot_height = 0.505 #meters
target_height = hopper_height - robot_height
hopper_width = 1.059942 #meters
l_margin = hopper_width / 2.0
g = 9.81 #m/s^2
L_max_limit = 6.0 #meters

def v_0(L, theta): #meters, degrees
    theta_rad = (theta / 180) * np.pi
    denominator = L * np.sin(2 * theta_rad) - target_height * np.cos(2 * theta_rad) - target_height
    
    valid = denominator > 0
    v = np.full_like(theta, np.nan, dtype=np.float64)
    v[valid] = L * np.sqrt(g / denominator[valid])
    return v

# --- Plot and UI Settings ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
plt.subplots_adjust(bottom=0.35)

L_init = 3.0
theta_init = 55.0
hmax_init = 4.0
theta_eval = np.linspace(theta_min_limit, theta_max_limit, 500)

L_min = max(0.1, L_init - l_margin)
L_M_init = L_init + l_margin

v0_min_vals = v_0(L_min, theta_eval)
v0_center_vals = v_0(L_init, theta_eval)
v0_max_vals = v_0(L_M_init, theta_eval)

# Calculate intersection analytically
# tan(theta) = 2*L*h / (L^2 - l^2)
theta_intersect_rad_init = np.arctan((2 * L_init * target_height) / (L_init**2 - l_margin**2))
theta_intersect_deg_init = np.degrees(theta_intersect_rad_init)

# --- Graph 1: v0 vs theta ---
line_min, = ax1.plot(theta_eval, v0_min_vals, 'r--', label='L - l (Front edge)')
line_center, = ax1.plot(theta_eval, v0_center_vals, 'g-', linewidth=2, label='L (Center)')
line_max, = ax1.plot(theta_eval, v0_max_vals, 'b--', label='L + l (Back edge)')

# Valid region
valid_fill_init = theta_eval >= theta_intersect_deg_init
fill_region = ax1.fill_between(theta_eval, v0_min_vals, v0_max_vals, where=valid_fill_init, color='gray', alpha=0.3, label='Valid Region')

# h_max limit line
theta_eval_rad = np.radians(theta_eval)
v0_hmax_vals = (1 / np.sin(theta_eval_rad)) * np.sqrt(2 * g * hmax_init)
line_hmax, = ax1.plot(theta_eval, v0_hmax_vals, 'k-', linewidth=1.5, label='Max Height Limit ($h_{max}$)')

# Analytical solution for optimal theta and v0
def calculate_optimal_trajectory(L, h_max, target_h):
    # alpha = h_max + sqrt(h_max * (h_max - h))
    alpha = h_max + np.sqrt(max(0, h_max * (h_max - target_h)))
    if alpha <= 0:
        return np.nan, np.nan
    
    # theta = arctan(2*alpha / L)
    theta_rad = np.arctan((2.0 * alpha) / L)
    theta_deg = np.degrees(theta_rad)
    
    # v0 = (sqrt(L^2 + 4*alpha^2) / (2*alpha)) * sqrt(2*g*h_max)
    v0_opt = (np.sqrt(L**2 + 4 * alpha**2) / (2.0 * alpha)) * np.sqrt(2 * g * h_max)
    
    return theta_deg, v0_opt

theta_opt_init, v0_opt_init = calculate_optimal_trajectory(L_init, hmax_init, target_height)

vline_theta = ax1.axvline(x=theta_opt_init, color='purple', linestyle=':', label='Optimal $\\theta$')
point_v0, = ax1.plot([theta_opt_init], [v0_opt_init], 'mo', markersize=8, label='Optimal $v_0$')

ax1.set_title(f"Required $v_0$ vs $\\theta$ for Target at L={L_init}m")
ax1.set_xlabel("Angle $\\theta$ (degrees)")
ax1.set_ylabel("Initial Velocity $v_0$ (m/s)")
ax1.set_ylim(v0_min_limit, v0_max_limit)
ax1.set_xlim(theta_min_limit, theta_max_limit)
ax1.legend()
ax1.grid(True)

# --- Graph 2: Trajectory Simulation ---
import matplotlib.patches as patches

ax2.set_title("Trajectory Simulation")
ax2.set_xlabel("Distance x (m)")
ax2.set_ylabel("Height y (m)")
ax2.set_xlim(0, 10)
ax2.set_ylim(0, 8)
ax2.grid(True)

# Target box
target_patch = patches.Rectangle((L_init - l_margin, 0), hopper_width, target_height, 
                                 linewidth=1, edgecolor='r', facecolor='none', hatch='//')
ax2.add_patch(target_patch)

# Trajectory
traj_line, = ax2.plot([], [], 'b-', linewidth=2, label='Trajectory')
ax2.plot(0, 0, 'ko', label='Shooter')
ax2.legend()

# Add Sliders
ax_L = plt.axes([0.15, 0.15, 0.65, 0.03])
ax_hmax = plt.axes([0.15, 0.08, 0.65, 0.03])

slider_L = Slider(ax_L, 'Distance L (m)', 0.2, L_max_limit, valinit=L_init)
slider_hmax = Slider(ax_hmax, '$h_{max}$ (m)', 2.0, 10.0, valinit=hmax_init)

def update(val):
    global fill_region
    L = slider_L.val
    h_max = slider_hmax.val
    
    L_m = max(0.1, L - l_margin)
    L_M = L + l_margin
    
    v0_m_vals = v_0(L_m, theta_eval)
    v0_c_vals = v_0(L, theta_eval)
    v0_M_vals = v_0(L_M, theta_eval)
    
    # Update h_max limit line
    v0_hmax_updated = (1 / np.sin(np.radians(theta_eval))) * np.sqrt(2 * g * h_max)
    line_hmax.set_ydata(v0_hmax_updated)
    
    line_min.set_ydata(v0_m_vals)
    line_center.set_ydata(v0_c_vals)
    line_max.set_ydata(v0_M_vals)
    
    # Calculate intersection for the selected L
    valid_denom = max(1e-5, (L**2 - l_margin**2))
    theta_intersect_rad = np.arctan((2 * L * target_height) / valid_denom)
    theta_intersect = np.degrees(theta_intersect_rad)
    
    fill_region.remove()
    valid_fill = theta_eval >= theta_intersect
    fill_region = ax1.fill_between(theta_eval, v0_m_vals, v0_M_vals, where=valid_fill, color='gray', alpha=0.3)
    
    # Calculate optimal theta and v0 directly from analytical solution
    theta_opt, selected_v0 = calculate_optimal_trajectory(L, h_max, target_height)
    
    vline_theta.set_xdata([theta_opt, theta_opt])
    
    if not np.isnan(selected_v0):
        point_v0.set_data([theta_opt], [selected_v0])
    else:
        point_v0.set_data([], [])
        
    ax1.set_title(f"Target at L={L:.2f}m | Optimal $\\theta$={theta_opt:.1f}deg | Chosen $v_0$={selected_v0:.2f} m/s")
    
    # Update trajectory plot
    target_patch.set_x(L - l_margin)
    
    if not np.isnan(selected_v0):
        t = np.linspace(0, 5, 500)
        theta_rad = np.radians(theta_opt)
        x = selected_v0 * np.cos(theta_rad) * t
        y = selected_v0 * np.sin(theta_rad) * t - 0.5 * g * t**2
        
        # Only show y >= 0
        valid_idx = y >= 0
        traj_line.set_data(x[valid_idx], y[valid_idx])
    else:
        traj_line.set_data([], [])
        
    fig.canvas.draw_idle()

slider_L.on_changed(update)
slider_hmax.on_changed(update)
plt.show()

