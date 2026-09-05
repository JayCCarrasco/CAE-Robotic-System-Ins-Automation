import csv
import json
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset


# ============================================================
# CONFIGURACIÓN
# ============================================================

CSV_FILE = "data/raw/balancing_robot_dataset.csv"
MODEL_FILE = "models/balancing_network_v1.pth"
NORMALIZATION_FILE = "models/normalization_v1.json"

BATCH_SIZE = 64
EPOCHS = 100
LEARNING_RATE = 1e-3

TRAIN_RATIO = 0.70
VAL_RATIO = 0.15


# ============================================================
# 1. CARGAR DATASET
# ============================================================

data = []

with open(CSV_FILE, newline="") as f:
    reader = csv.DictReader(f)

    for row in reader:

        theta = float(row["theta"])
        theta_dot = float(row["theta_dot"])
        x_dot = float(row["x_dot"])

        effort = float(row["effort"])

        data.append([
            theta,
            theta_dot,
            x_dot,
            effort
        ])


data = np.array(data, dtype=np.float32)

print("Dataset:", data.shape)


# ============================================================
# 2. SEPARAR ENTRADAS Y OBJETIVO
# ============================================================

# Entradas de la red:
#
# X = [theta, theta_dot, x_dot]
#
# NO utilizamos:
# - timestamp
# - x

X = data[:, 0:3]
y = data[:, 3:4]


# ============================================================
# 3. DIVISIÓN TEMPORAL
# ============================================================

N = len(X)

train_end = int(N * TRAIN_RATIO)
val_end = int(N * (TRAIN_RATIO + VAL_RATIO))

X_train = X[:train_end]
y_train = y[:train_end]

X_val = X[train_end:val_end]
y_val = y[train_end:val_end]

X_test = X[val_end:]
y_test = y[val_end:]


print("\nDivisión del dataset:")
print("Train:", len(X_train))
print("Validation:", len(X_val))
print("Test:", len(X_test))


# ============================================================
# 4. NORMALIZACIÓN
# ============================================================

# Las estadísticas se calculan SOLO usando TRAIN.

X_mean = X_train.mean(axis=0)
X_std = X_train.std(axis=0)

y_mean = y_train.mean(axis=0)
y_std = y_train.std(axis=0)

# Evitar división por cero
X_std[X_std == 0] = 1.0
y_std[y_std == 0] = 1.0


X_train = (X_train - X_mean) / X_std
X_val = (X_val - X_mean) / X_std
X_test = (X_test - X_mean) / X_std

y_train = (y_train - y_mean) / y_std
y_val = (y_val - y_mean) / y_std
y_test = (y_test - y_mean) / y_std


# ============================================================
# 5. GUARDAR NORMALIZACIÓN
# ============================================================

normalization = {
    "X_mean": X_mean.tolist(),
    "X_std": X_std.tolist(),
    "y_mean": y_mean.tolist(),
    "y_std": y_std.tolist()
}

with open(NORMALIZATION_FILE, "w") as f:
    json.dump(normalization, f, indent=4)

print("\nNormalización guardada en:")
print(NORMALIZATION_FILE)


# ============================================================
# 6. CONVERTIR A TENSORES
# ============================================================

X_train = torch.tensor(X_train)
y_train = torch.tensor(y_train)

X_val = torch.tensor(X_val)
y_val = torch.tensor(y_val)

X_test = torch.tensor(X_test)
y_test = torch.tensor(y_test)


# ============================================================
# 7. DATALOADERS
# ============================================================

train_dataset = TensorDataset(X_train, y_train)
val_dataset = TensorDataset(X_val, y_val)
test_dataset = TensorDataset(X_test, y_test)

train_loader = DataLoader(
    train_dataset,
    batch_size=BATCH_SIZE,
    shuffle=True
)

val_loader = DataLoader(
    val_dataset,
    batch_size=BATCH_SIZE,
    shuffle=False
)

test_loader = DataLoader(
    test_dataset,
    batch_size=BATCH_SIZE,
    shuffle=False
)


# ============================================================
# 8. RED NEURONAL
# ============================================================

class BalancingNetwork(nn.Module):

    def __init__(self):
        super().__init__()

        self.network = nn.Sequential(
            nn.Linear(3, 32),
            nn.ReLU(),

            nn.Linear(32, 32),
            nn.ReLU(),

            nn.Linear(32, 1)
        )

    def forward(self, x):
        return self.network(x)


model = BalancingNetwork()

print("\nModelo:")
print(model)


# ============================================================
# 9. FUNCIÓN DE PÉRDIDA Y OPTIMIZADOR
# ============================================================

criterion = nn.MSELoss()

optimizer = torch.optim.Adam(
    model.parameters(),
    lr=LEARNING_RATE
)


# ============================================================
# 10. ENTRENAMIENTO
# ============================================================

print("\nComenzando entrenamiento...\n")

for epoch in range(EPOCHS):

    # --------------------------------------------------------
    # TRAIN
    # --------------------------------------------------------

    model.train()

    train_loss = 0.0

    for X_batch, y_batch in train_loader:

        optimizer.zero_grad()

        prediction = model(X_batch)

        loss = criterion(
            prediction,
            y_batch
        )

        loss.backward()

        optimizer.step()

        train_loss += loss.item()

    train_loss /= len(train_loader)


    # --------------------------------------------------------
    # VALIDATION
    # --------------------------------------------------------

    model.eval()

    val_loss = 0.0

    with torch.no_grad():

        for X_batch, y_batch in val_loader:

            prediction = model(X_batch)

            loss = criterion(
                prediction,
                y_batch
            )

            val_loss += loss.item()

    val_loss /= len(val_loader)


    # --------------------------------------------------------
    # MOSTRAR RESULTADOS
    # --------------------------------------------------------

    if (epoch + 1) % 10 == 0:

        print(
            f"Epoch [{epoch + 1:3d}/{EPOCHS}] "
            f"Train Loss: {train_loss:.6f} "
            f"Val Loss: {val_loss:.6f}"
        )


# ============================================================
# 11. EVALUACIÓN FINAL EN TEST
# ============================================================

model.eval()

test_loss = 0.0

with torch.no_grad():

    for X_batch, y_batch in test_loader:

        prediction = model(X_batch)

        loss = criterion(
            prediction,
            y_batch
        )

        test_loss += loss.item()

test_loss /= len(test_loader)


print("\n========================================")
print("RESULTADO FINAL")
print("========================================")
print(f"Test Loss: {test_loss:.6f}")


# ============================================================
# 12. GUARDAR MODELO
# ============================================================

torch.save(
    model.state_dict(),
    MODEL_FILE
)

print("\nModelo guardado en:")
print(MODEL_FILE)