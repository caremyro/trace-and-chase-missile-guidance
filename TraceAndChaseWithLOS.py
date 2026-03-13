import numpy as np
import matplotlib
matplotlib.use('TkAgg') #Pour lancer avec WSL
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# ============================================================================
# Variables fixes
# ============================================================================
targ_vel            = 750
miss_vel            = 1200
tmax                = 90
dt                  = 0.001
animation_interval  = 5
kill_dist           = 20
N                   = 4.0
max_accel           = 500.0

# ============================================================================
# Paramètres aléatoires pour la position initiale et la trajectoire de la cible, et la position de lancement du missile
# ============================================================================
rng = np.random.default_rng()

altitude_avion     = rng.uniform(8000, 15000)
cap_initial        = rng.uniform(0, 2 * np.pi)

aircraft_start_loc = np.array([
    rng.uniform(-20000, 20000),
    rng.uniform(-20000, 20000),
    altitude_avion
])

Straight_time      = rng.uniform(15, 30)
curve_time         = rng.uniform(15, 25)
Straight_time2     = rng.uniform(15, 30)
turn_angle         = rng.choice([-1, 1]) * rng.uniform(np.pi/3, np.pi)
yz_angle           = rng.uniform(-np.pi/8, np.pi/8)
climb_rate_curve   = rng.uniform(-0.002, 0.002)

missile_start_loc  = np.array([
    rng.uniform(-30000, 30000),
    rng.uniform(-30000, 30000),
    0.0
])

missile_launch_time = rng.uniform(3, 12)

print("=" * 50)
print(f"Avion    : ({aircraft_start_loc[0]:.0f}, {aircraft_start_loc[1]:.0f}, {aircraft_start_loc[2]:.0f}) m")
print(f"Cap      : {np.degrees(cap_initial):.1f}°")
print(f"Virage   : {np.degrees(turn_angle):.1f}°")
print(f"Missile  : ({missile_start_loc[0]:.0f}, {missile_start_loc[1]:.0f}, 0) m")
print(f"Lancement: t = {missile_launch_time:.2f}s")
print("=" * 50)

# ============================================================================
# Variables globales segments
# ============================================================================
curve_start_x = curve_start_y = curve_start_z = None
curve_initialized   = False
straight2_start_x = straight2_start_y = straight2_start_z = None
straight2_initialized = False
radius   = (targ_vel * curve_time) / turn_angle
center_x = center_y = center_z = None

# ============================================================================
# TRAJECTOIRE CIBLE
# ============================================================================
def target_location(t, target_states):
    global curve_start_x, curve_start_y, curve_start_z, curve_initialized
    global straight2_start_x, straight2_start_y, straight2_start_z, straight2_initialized
    global center_x, center_y, center_z

    dx0 = np.cos(cap_initial)
    dy0 = np.sin(cap_initial)

    if 0 <= t <= Straight_time:
        x = aircraft_start_loc[0] + targ_vel * dx0 * t
        y = aircraft_start_loc[1] + targ_vel * dy0 * t
        z = aircraft_start_loc[2]

    elif Straight_time < t <= Straight_time + curve_time:
        tc = t - Straight_time
        if not curve_initialized:
            if len(target_states) > 0:
                curve_start_x = target_states[-1, 0]
                curve_start_y = target_states[-1, 1]
                curve_start_z = target_states[-1, 2]
            else:
                curve_start_x = aircraft_start_loc[0] + targ_vel * dx0 * Straight_time
                curve_start_y = aircraft_start_loc[1] + targ_vel * dy0 * Straight_time
                curve_start_z = aircraft_start_loc[2]
            sign = np.sign(turn_angle)
            center_x = curve_start_x + abs(radius) * (-sign * dy0)
            center_y = curve_start_y + abs(radius) * ( sign * dx0)
            center_z = curve_start_z
            curve_initialized = True
        frac      = tc / curve_time
        angle_now = frac * turn_angle
        vx0 =  np.sign(turn_angle) * dy0
        vy0 = -np.sign(turn_angle) * dx0
        cos_a, sin_a = np.cos(angle_now), np.sin(angle_now)
        vx = vx0 * cos_a - vy0 * sin_a
        vy = vx0 * sin_a + vy0 * cos_a

        x = center_x + abs(radius) * vx
        y = center_y + abs(radius) * vy
        z = curve_start_z \
            + np.sin(yz_angle + np.pi/2) * targ_vel**2 \
            * (1 - np.cos(np.pi * tc / curve_time)) * climb_rate_curve

    elif Straight_time + curve_time < t <= Straight_time + curve_time + Straight_time2:
        if not straight2_initialized:
            if len(target_states) > 0:
                straight2_start_x = target_states[-1, 0]
                straight2_start_y = target_states[-1, 1]
                straight2_start_z = target_states[-1, 2]
            else:
                straight2_start_x = curve_start_x
                straight2_start_y = curve_start_y
                straight2_start_z = curve_start_z
            straight2_initialized = True

        ts        = t - (Straight_time + curve_time)
        cap_final = cap_initial + turn_angle
        x = straight2_start_x + targ_vel * ts * np.cos(cap_final)
        y = straight2_start_y + targ_vel * ts * np.sin(cap_final)
        z = straight2_start_z

    else:
        if straight2_initialized:
            cap_final = cap_initial + turn_angle
            x = straight2_start_x + targ_vel * Straight_time2 * np.cos(cap_final)
            y = straight2_start_y + targ_vel * Straight_time2 * np.sin(cap_final)
            z = straight2_start_z
        else:
            x = aircraft_start_loc[0] + targ_vel * dx0 * Straight_time
            y = aircraft_start_loc[1] + targ_vel * dy0 * Straight_time
            z = aircraft_start_loc[2]

    return np.array([x, y, z])

# ============================================================================
# GÉNÉRATION TRAJECTOIRE CIBLE
# ============================================================================
curve_initialized     = False
straight2_initialized = False

times    = np.arange(0, tmax, dt)
n_points = len(times)

target_states = np.zeros((n_points, 3))
for i in range(n_points):
    target_states[i] = target_location(times[i], target_states[:i])

print(f"Trajectoire cible générée : {n_points} points sur {tmax:.1f}s")
print(f"Début virage    : ({curve_start_x:.1f}, {curve_start_y:.1f}, {curve_start_z:.1f})")
print(f"Début segment 2 : ({straight2_start_x:.1f}, {straight2_start_y:.1f}, {straight2_start_z:.1f})")

# ============================================================================
# GÉNÉRATION TRAJECTOIRE MISSILE — Navigation Proportionnelle - LOS aléatoire
# ============================================================================
missile_states     = np.zeros((n_points, 3))
missile_states[0]  = missile_start_loc
missile_launched   = False
intercepted        = False
intercept_index    = None
missile_vel_vector = None

for i in range(1, n_points):
    t = times[i]

    if t >= missile_launch_time and not missile_launched:
        missile_launched = True
        los0 = target_states[i] - missile_states[i-1]
        missile_vel_vector = los0 / np.linalg.norm(los0) * miss_vel
        print(f"Missile lancé à t={t:.3f}s")

    if not missile_launched:
        missile_states[i] = missile_start_loc
        continue

    if intercepted:
        missile_states[i] = missile_states[i-1]
        continue

    los_curr = target_states[i] - missile_states[i-1]
    distance = np.linalg.norm(los_curr)
    u_los    = los_curr / distance

    v_tgt      = (target_states[i] - target_states[i-1]) / dt
    v_approach = missile_vel_vector - v_tgt
    Vc         = np.dot(v_approach, u_los)

    if distance > 1.0 and Vc > 0:
        omega_los = np.cross(u_los, v_approach) / distance
        accel_cmd = N * Vc * np.cross(u_los, omega_los)
        a_norm = np.linalg.norm(accel_cmd)
        if a_norm > max_accel:
            accel_cmd = accel_cmd / a_norm * max_accel
        missile_vel_vector += accel_cmd * dt

    spd = np.linalg.norm(missile_vel_vector)
    if spd > 0:
        missile_vel_vector = missile_vel_vector / spd * miss_vel

    new_pos = missile_states[i-1] + missile_vel_vector * dt
    if new_pos[2] < 0:
        new_pos[2] = 0.0
        missile_vel_vector[2] = max(0.0, missile_vel_vector[2])

    missile_states[i] = new_pos

    if distance < kill_dist:
        intercept_index, intercepted = i, True
        print(f"INTERCEPTION à t={t:.2f}s  distance={distance:.2f}m")

final_distance = np.linalg.norm(target_states[-1] - missile_states[-1])
print(f"Distance finale : {final_distance:.1f}m")
if not intercepted:
    print("Pas d'interception — augmente miss_vel, N ou max_accel")

# ============================================================================
# AFFICHAGE 3D
# ============================================================================
fig = plt.figure(figsize=(14, 10))
ax  = fig.add_subplot(111, projection='3d')

all_points  = np.vstack([target_states, missile_states])
x_center    = (np.max(all_points[:,0]) + np.min(all_points[:,0])) / 2
y_center    = (np.max(all_points[:,1]) + np.min(all_points[:,1])) / 2
z_center    = (np.max(all_points[:,2]) + np.min(all_points[:,2])) / 2
plot_radius = max(np.ptp(all_points[:,0]), np.ptp(all_points[:,1]), np.ptp(all_points[:,2])) / 2 * 1.1

ax.set_xlim(x_center - plot_radius, x_center + plot_radius)
ax.set_ylim(y_center - plot_radius, y_center + plot_radius)
ax.set_zlim(z_center - plot_radius, z_center + plot_radius)
ax.set_box_aspect([1, 1, 1])
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Missile-Aircraft Simulation — PN / LOS aléatoire')
ax.grid(True)
ax.view_init(elev=20, azim=45)

target_point,  = ax.plot([], [], [], 'bo', markersize=10, label='Cible')
target_trail,  = ax.plot([], [], [], 'b-', linewidth=2,   alpha=0.5, label='Trace cible')
missile_point, = ax.plot([], [], [], 'ro', markersize=8,  label='Missile')
missile_trail, = ax.plot([], [], [], 'r-', linewidth=1.5, alpha=0.5, label='Trace missile')
time_text     = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
speed_text    = ax.text2D(0.02, 0.90, '', transform=ax.transAxes, fontsize=10)
distance_text = ax.text2D(0.02, 0.85, '', transform=ax.transAxes, fontsize=10)

ax.scatter(*target_states[0],  c='green',  s=100, marker='s', label='Départ cible')
ax.scatter(*missile_start_loc, c='orange', s=100, marker='^', label='Départ missile')
if intercept_index is not None:
    ax.scatter(*target_states[intercept_index], c='red', s=200, marker='*', label='Intercept')

xlim = ax.get_xlim()
ylim = ax.get_ylim()
xx, yy = np.meshgrid(
    np.linspace(xlim[0], xlim[1], 3),
    np.linspace(ylim[0], ylim[1], 3)
)
ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.08, color='green')
ax.plot([missile_start_loc[0], missile_start_loc[0]],
        [missile_start_loc[1], missile_start_loc[1]],
        [0, 0], 'o', color='orange', markersize=8)

ax.set_zlim(0, z_center + plot_radius)

ax.legend()

# ============================================================================
# ANIMATION
# ============================================================================
def init():
    for artist in [target_point, target_trail, missile_point, missile_trail]:
        artist.set_data([], [])
        artist.set_3d_properties([])
    time_text.set_text('')
    speed_text.set_text('')
    distance_text.set_text('')
    return target_point, target_trail, missile_point, missile_trail, time_text, speed_text, distance_text

def update(frame):
    target_point.set_data([target_states[frame,0]], [target_states[frame,1]])
    target_point.set_3d_properties([target_states[frame,2]])
    target_trail.set_data(target_states[:frame+1,0], target_states[:frame+1,1])
    target_trail.set_3d_properties(target_states[:frame+1,2])

    missile_point.set_data([missile_states[frame,0]], [missile_states[frame,1]])
    missile_point.set_3d_properties([missile_states[frame,2]])
    missile_trail.set_data(missile_states[:frame+1,0], missile_states[:frame+1,1])
    missile_trail.set_3d_properties(missile_states[:frame+1,2])

    if frame > 0:
        speed = np.linalg.norm(target_states[frame] - target_states[frame-1]) / dt
    else:
        speed = targ_vel

    distance = np.linalg.norm(target_states[frame] - missile_states[frame])
    time_text.set_text(f'Time     = {times[frame]:.2f} s')
    speed_text.set_text(f'Cible    = {speed:.1f} m/s')
    distance_text.set_text(f'Distance = {distance:.1f} m')

    return target_point, target_trail, missile_point, missile_trail, time_text, speed_text, distance_text

frame_skip = max(1, n_points // 500)
frames     = range(0, n_points, frame_skip)
print(f"Animation : {len(frames)} frames")

anim = FuncAnimation(fig, update, frames=frames, init_func=init,
                     blit=False, interval=animation_interval, repeat=True)
plt.show()