import pandas as pd
import numpy as np

# Zeit und 7 Gelenkwinkel
t = np.linspace(0, 1, 101)
angles = np.stack([np.sin(2 * np.pi * t + phase) * 0.5 for phase in np.linspace(0, np.pi, 7)], axis=1)

# DataFrame bauen
data = pd.DataFrame(data=np.column_stack((t, angles)),
                    columns=["t", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6", "theta7"])

# Speichern
data.to_csv("gelenkwinkel.csv", index=False)
