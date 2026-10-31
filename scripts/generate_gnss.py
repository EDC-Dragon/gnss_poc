
import csv
from pathlib import Path
from datetime import datetime
import numpy as np
import gnss_lib_py as glp

SCRIPT_DIR = Path(__file__).resolve().parent

# Configuration
C = 299_792_458.0
EPOCH_COUNT = 1000
DT = 1.0
GPS_START_MS = 1_450_000_000_000

SIGMA_M = 5.0
CLOCK_BIAS_M = 150.0

INITIAL_POSITION = np.array([
    4_200_000.0, 170_000.0, 4_780_000.0
])

RECEIVER_VELOCITY = np.array([
    100.0, 100.0, 100.0
])

# Orbites circulaires simplifiées.
# Période illustrative : 12 heures.
ORBIT_OMEGA = 2.0 * np.pi / (12.0 * 3600.0)

rng = np.random.default_rng(42)


def normalize(v):
    return v / np.linalg.norm(v)


# Construire six positions satellites initiales.
up = normalize(INITIAL_POSITION)
east = normalize(np.cross([0.0, 0.0, 1.0], up))
north = normalize(np.cross(up, east))

directions = np.array([
    up,
    up + east,
    up - east,
    up + north,
    up - north,
    up + east + north,
])

directions = np.array([normalize(d) for d in directions])

initial_sat_positions = (
    INITIAL_POSITION + 20_000_000.0 * directions
)

# Chaque satellite tourne dans un plan orbital simplifié.
orbital_axes = np.array([
    normalize(np.cross(p, [0.0, 0.0, 1.0]))
    for p in initial_sat_positions
])


def satellite_states(t):
    angle = ORBIT_OMEGA * t
    positions = []
    velocities = []

    for p0, axis in zip(initial_sat_positions, orbital_axes):
        # Rotation autour de l'axe orbital (formule de Rodrigues).
        position = (
            p0 * np.cos(angle)
            + np.cross(axis, p0) * np.sin(angle)
        )

        velocity = ORBIT_OMEGA * np.cross(axis, position)

        positions.append(position)
        velocities.append(velocity)

    positions = np.array(positions)
    velocities = np.array(velocities)

    sv = glp.NavData()

    sv["gnss_id"] = np.array(["gps"] * 6)
    sv["sv_id"] = np.arange(1, 7)

    for i, axis in enumerate(["x", "y", "z"]):
        sv[f"{axis}_sv_m"] = positions[:, i]
        sv[f"v{axis}_sv_mps"] = velocities[:, i]

    return sv


def receiver_state(position):
    state = glp.NavData()

    for i, axis in enumerate(["x", "y", "z"]):
        state[f"{axis}_rx_m"] = np.array([position[i]])
        state[f"v{axis}_rx_mps"] = np.array([RECEIVER_VELOCITY[i]])

    state["b_rx_m"] = np.array([CLOCK_BIAS_M])
    state["b_dot_rx_mps"] = np.array([0.0])

    return state

 
output_folder = SCRIPT_DIR / ".." / "simulation_data"
timestamp = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
batch_folder = output_folder / f"batch_{timestamp}"
batch_folder.mkdir(exist_ok=True, parents=True)
measurement_path = batch_folder / "measurements.csv" 

with (
    measurement_path.open("w", newline="") as mf, 
):
    measurements_writer = csv.writer(mf) 
 

    for epoch in range(EPOCH_COUNT):
        t = epoch * DT

        reception_time_s = t 

        gps_millis = GPS_START_MS + round(t * 1000)

        # Position réelle du récepteur à l'époque t.
        receiver_position = (
            INITIAL_POSITION + RECEIVER_VELOCITY * t
        )

        state = receiver_state(receiver_position)
        sv = satellite_states(t)

        measurements, visible_sv = glp.simulate_measures(
            gps_millis=gps_millis,
            state=state,
            sv_posvel=sv,
            noise_dict={
                "prange_sigma": SIGMA_M,
                "doppler_sigma": 1.0,
            },
            rng=rng,
            el_mask=5.0,
        )

        # Export des observations.
        for i in range(len(measurements)):
            satellite_position = np.array([
                float(visible_sv["x_sv_m", i]),
                float(visible_sv["y_sv_m", i]),
                float(visible_sv["z_sv_m", i]),
            ])

            # Distance géométrique satellite-récepteur.
            geometric_distance_m = np.linalg.norm(
                satellite_position - receiver_position
            )

            # Temps de propagation du signal.
            propagation_time_s = geometric_distance_m / C

            # Instant d'émission du signal.
            transmission_time_s = (
                reception_time_s - propagation_time_s
            )

            if epoch == 0:  
                if i == 0:
                    measurements_writer.writerow([
                        "transmission_time_s",
                        "reception_time_s",
                        "satellite_id",
                        "satellite_position_x",
                        "satellite_position_y",
                        "satellite_position_z",
                        "pseudorange_m",
                        "sigma_m",
                        "receiver_position_x",
                        "receiver_position_y",
                        "receiver_position_z",
                        "receiver_velocity_x",
                        "receiver_velocity_y",
                        "receiver_velocity_z",
                    ])
            measurements_writer.writerow([
                transmission_time_s,
                reception_time_s,
                int(measurements["sv_id", i]),
                *satellite_position,
                float(measurements["raw_pr_m", i]),
                SIGMA_M, 
                *receiver_position, 
                *RECEIVER_VELOCITY, 
            ])
 
        if epoch % 100 == 0:
            print(
                f"t={t:.0f}s | "
                f"satellites visibles={len(measurements)}"
            )

print(f"Mesures : {measurement_path}") 
