import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def plot_jsbsim_log(csv_path: str | Path):
    """
    Quick visualization for system identification logs.
    Expects the CSV created by SystemIdentifierSingleCase.run().
    """

    df = pd.read_csv(csv_path)

    # --- Derived quantities
    t = df["t"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("JSBSim System Identification Log", fontsize=14, weight="bold")

    # 1️⃣ Attitude angles
    ax = axes[0]
    ax.plot(t, df["theta_rad"], label="Pitch θ [rad]")
    ax.plot(t, df["phi_rad"], label="Bank φ [rad]")
    ax.set_ylabel("Attitude [rad]")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    # 2️⃣ Body rates
    ax = axes[1]
    ax.plot(t, df["q_radps"], label="q [rad/s]")
    ax.plot(t, df["p_radps"], label="p [rad/s]")
    ax.plot(t, df["r_radps"], label="r [rad/s]")
    ax.set_ylabel("Body rates [rad/s]")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    # 3️⃣ Control commands
    ax = axes[2]
    ax.plot(t, df["de_cmd"], label="Elevator cmd")
    ax.plot(t, df["da_cmd"], label="Aileron cmd")
    ax.plot(t, df["dr_cmd"], label="Rudder cmd")
    ax.set_ylabel("Command [norm]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


if __name__ == "__main__":
    # Example usage (adjust path as needed)
    plot_jsbsim_log("output/jsbsim/toy/system_id_log.csv")
