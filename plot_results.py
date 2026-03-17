import pandas as pd
import matplotlib.pyplot as plt

# Load your results
df = pd.read_csv("results.csv")

# Plot zone switches
plt.figure()
plt.plot(df["nodes"], df["baseline_switches"], marker='o', label="Baseline")
plt.plot(df["nodes"], df["constraint_switches"], marker='o', label="Constraint")

plt.xlabel("Number of Nodes")
plt.ylabel("Zone Switches")
plt.title("Zone Switching Comparison")
plt.legend()

# Save image
plt.savefig("zone_switches.png")

# Show graph
plt.show()