import csv
import numpy as np

CSV_FILE = "data/raw/balancing_robot_dataset.csv"

data = []

with open(CSV_FILE, newline="") as f:
    reader = csv.DictReader(f)

    for row in reader:
        data.append([
            int(row["timestamp"]),
            float(row["theta"]),
            float(row["theta_dot"]),
            float(row["x_dot"]),
            float(row["effort"])
        ])

data = np.array(data)

timestamp = data[:, 0]
theta = data[:, 1]
theta_dot = data[:, 2]
x_dot = data[:, 3]
effort = data[:, 4]

N = len(data)

print("========================================")
print("ANÁLISIS DEL DATASET")
print("========================================")

print(f"Filas: {N}")

dt = np.diff(timestamp) * 1e-9

print(f"\nFrecuencia:")
print(f"  dt medio: {dt.mean() * 1000:.3f} ms")
print(f"  frecuencia media: {1 / dt.mean():.2f} Hz")

# --------------------------------------------------
# Dividir temporalmente en 10 bloques
# --------------------------------------------------

print("\n========================================")
print("DISTRIBUCIÓN TEMPORAL")
print("========================================")

blocks = np.array_split(data, 10)

for i, block in enumerate(blocks):

    theta_b = block[:, 1]
    theta_dot_b = block[:, 2]
    x_dot_b = block[:, 3]
    effort_b = block[:, 4]

    print(
        f"\nBloque {i+1:2d}: "
        f"{len(block):4d} muestras"
    )

    print(
        f"  theta     [{theta_b.min(): .4f}, {theta_b.max(): .4f}]"
    )

    print(
        f"  theta_dot [{theta_dot_b.min(): .4f}, {theta_dot_b.max(): .4f}]"
    )

    print(
        f"  x_dot     [{x_dot_b.min(): .4f}, {x_dot_b.max(): .4f}]"
    )

    print(
        f"  effort    [{effort_b.min(): .4f}, {effort_b.max(): .4f}]"
    )

# --------------------------------------------------
# Estados cercanos al equilibrio
# --------------------------------------------------

print("\n========================================")
print("MUESTRAS CERCA DEL EQUILIBRIO")
print("========================================")

near_equilibrium = (
    (np.abs(theta) < 0.005) &
    (np.abs(theta_dot) < 0.05) &
    (np.abs(x_dot) < 0.05)
)

percentage = 100 * np.mean(near_equilibrium)

print(
    f"Muestras cerca del equilibrio: "
    f"{near_equilibrium.sum()} / {N}"
)

print(f"Porcentaje: {percentage:.2f}%")

# --------------------------------------------------
# Muestras de perturbación
# --------------------------------------------------

print("\n========================================")
print("MUESTRAS CON PERTURBACIÓN")
print("========================================")

perturbed = (
    (np.abs(theta) > 0.01) |
    (np.abs(theta_dot) > 0.15) |
    (np.abs(x_dot) > 0.10)
)

percentage = 100 * np.mean(perturbed)

print(
    f"Muestras perturbadas: "
    f"{perturbed.sum()} / {N}"
)

print(f"Porcentaje: {percentage:.2f}%")

# --------------------------------------------------
# Cambios bruscos de estado
# --------------------------------------------------

print("\n========================================")
print("CAMBIOS BRUSCOS")
print("========================================")

theta_change = np.abs(np.diff(theta))
theta_dot_change = np.abs(np.diff(theta_dot))
x_dot_change = np.abs(np.diff(x_dot))

print(f"Δtheta máximo:     {theta_change.max():.5f}")
print(f"Δtheta_dot máximo: {theta_dot_change.max():.5f}")
print(f"Δx_dot máximo:     {x_dot_change.max():.5f}")

print("\nAnálisis terminado.")