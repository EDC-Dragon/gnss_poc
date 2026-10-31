
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
import gnss_lib_py as glp

# Position et vitesse réelles du récepteur.
rx = np.array([4_200_000.0, 170_000.0, 4_780_000.0])
velocity = np.array([100.0, 100.0, 100.0])

state = glp.NavData()

for name, value in zip(
    ["x_rx_m", "y_rx_m", "z_rx_m"],
    rx
):
    state[name] = np.array([value])

for name, value in zip(
    ["vx_rx_mps", "vy_rx_mps", "vz_rx_mps"],
    velocity
):
    state[name] = np.array([value])

state["b_rx_m"] = np.array([150.0])
state["b_dot_rx_mps"] = np.array([0.0])

# Construire six directions différentes au-dessus du récepteur.
up = rx / np.linalg.norm(rx)

east = np.cross(np.array([0.0, 0.0, 1.0]), up)
east /= np.linalg.norm(east)

north = np.cross(up, east)

directions = np.array([
    up,
    up + east,
    up - east,
    up + north,
    up - north,
    up + east + north,
])

directions /= np.linalg.norm(directions, axis=1)[:, None]

# Satellites synthétiques à environ 20 000 km du récepteur.
sat_positions = rx + 20_000_000.0 * directions

sv = glp.NavData()
sv["gnss_id"] = np.array(["gps"] * 6)
sv["sv_id"] = np.arange(1, 7)

for axis, name in enumerate(["x_sv_m", "y_sv_m", "z_sv_m"]):
    sv[name] = sat_positions[:, axis]

for name in ["vx_sv_mps", "vy_sv_mps", "vz_sv_mps"]:
    sv[name] = np.zeros(6)

# Générer les observations.
measurements, satellites = glp.simulate_measures(
    gps_millis=1_450_000_000_000,
    state=state,
    sv_posvel=sv,
    noise_dict={
        "prange_sigma": 5.0,
        "doppler_sigma": 1.0,
    },
    rng=np.random.default_rng(42),
    el_mask=5.0,
)

print(measurements)

# Export compatible avec le futur lecteur C11.
SCRIPT_DIR = Path(__file__).resolve().parent
output_folder = SCRIPT_DIR / ".." / "simulation_data"
output_folder.mkdir(exist_ok=True, parents=True)
timestamp = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
output = output_folder / f"measurements_{timestamp}.csv"

with output.open("w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow([
        "time_s", "satellite_id",
        "sat_x", "sat_y", "sat_z",
        "pseudorange_m", "sigma_m"
    ])

    for i in range(len(measurements)):
        writer.writerow([
            0.0,
            int(measurements["sv_id", i]),
            float(satellites["x_sv_m", i]),
            float(satellites["y_sv_m", i]),
            float(satellites["z_sv_m", i]),
            float(measurements["raw_pr_m", i]),
            5.0,
        ])

print(f"Export : {output}")

















import csv
from pathlib import Path

import numpy as np
import gnss_lib_py as glp

SCRIPT_DIR = Path(__file__).resolve().parent

# Configuration
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
truth_path = batch_folder / "ground_truth.csv"

with (
    measurement_path.open("w", newline="") as mf,
    truth_path.open("w", newline="") as tf,
):
    measurements_writer = csv.writer(mf)
    truth_writer = csv.writer(tf)

    measurements_writer.writerow([
        "time_s", "satellite_id",
        "sat_x", "sat_y", "sat_z",
        "pseudorange_m", "sigma_m"
    ])

    truth_writer.writerow([
        "time_s", "x", "y", "z",
        "vx", "vy", "vz"
    ])

    for epoch in range(EPOCH_COUNT):
        t = epoch * DT
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
            measurements_writer.writerow([
                t,
                int(measurements["sv_id", i]),
                float(visible_sv["x_sv_m", i]),
                float(visible_sv["y_sv_m", i]),
                float(visible_sv["z_sv_m", i]),
                float(measurements["raw_pr_m", i]),
                SIGMA_M,
            ])

        # Export de la trajectoire exacte.
        truth_writer.writerow([
            t,
            *receiver_position,
            *RECEIVER_VELOCITY,
        ])

        if epoch % 100 == 0:
            print(
                f"t={t:.0f}s | "
                f"satellites visibles={len(measurements)}"
            )

print(f"Mesures : {measurement_path}")
print(f"Vérité terrain : {truth_path}")
