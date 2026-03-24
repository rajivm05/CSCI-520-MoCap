import matplotlib.pyplot as plt
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "output")

def parse_joint_channel(amc_path, joint_name, channel_index, frame_start, frame_end):
    """
    Extracts a specific channel value for a joint across a range of frames.
    channel_index is 0-based index into the values after the joint name.
    Returns dict mapping frame number -> value.
    """
    data = {}
    current_frame = None

    with open(amc_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith(':'):
                continue

            parts = line.split()

            # check if this line is a frame number (single integer on a line)
            if len(parts) == 1:
                try:
                    current_frame = int(parts[0])
                    continue
                except ValueError:
                    pass

            # check if this line is the joint we want
            if parts[0] == joint_name and current_frame is not None:
                if frame_start <= current_frame <= frame_end:
                    data[current_frame] = float(parts[1 + channel_index])

    return data


def plot_comparison(title, xlabel, ylabel, frame_range, input_data, method1_data, method2_data,
                    input_label, method1_label, method2_label, save_path):
    """Generates a comparison plot with 3 curves."""
    fig, ax = plt.subplots(figsize=(12, 5))

    frames = sorted(frame_range)
    input_vals = [input_data.get(fr, 0) for fr in frames]
    m1_vals = [method1_data.get(fr, 0) for fr in frames]
    m2_vals = [method2_data.get(fr, 0) for fr in frames]

    ax.plot(frames, input_vals, 'k-', linewidth=1.5, label=input_label, alpha=0.8)
    ax.plot(frames, m1_vals, 'b--', linewidth=1.2, label=method1_label, alpha=0.85)
    ax.plot(frames, m2_vals, 'r-.', linewidth=1.2, label=method2_label, alpha=0.85)

    ax.set_title(title, fontsize=13)
    ax.set_xlabel(xlabel, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.close()
    print(f"Saved: {save_path}")


if __name__ == "__main__":
    # file paths
    original = os.path.join(BASE_DIR, "131_04-dance.amc")
    le_file  = os.path.join(OUTPUT_DIR, "dance-le-N20.amc")
    be_file  = os.path.join(OUTPUT_DIR, "dance-be-N20.amc")
    lq_file  = os.path.join(OUTPUT_DIR, "dance-lq-N20.amc")
    bq_file  = os.path.join(OUTPUT_DIR, "dance-bq-N20.amc")

    # ---- Graphs 1 & 2: lfemur, X rotation (channel 0), frames 600-800 ----
    joint_1 = "lfemur"
    chan_1 = 0  # rx is the first channel
    fs1, fe1 = 600, 800

    input_lfemur  = parse_joint_channel(original, joint_1, chan_1, fs1, fe1)
    le_lfemur     = parse_joint_channel(le_file,  joint_1, chan_1, fs1, fe1)
    be_lfemur     = parse_joint_channel(be_file,  joint_1, chan_1, fs1, fe1)
    lq_lfemur     = parse_joint_channel(lq_file,  joint_1, chan_1, fs1, fe1)
    bq_lfemur     = parse_joint_channel(bq_file,  joint_1, chan_1, fs1, fe1)

    frame_range_1 = range(fs1, fe1 + 1)

    # Graph 1: Linear Euler vs Bezier Euler
    plot_comparison(
        title="Graph 1: lfemur X-rotation — Linear Euler vs Bezier Euler (N=20)",
        xlabel="Frame", ylabel="Angle (degrees)",
        frame_range=frame_range_1,
        input_data=input_lfemur, method1_data=le_lfemur, method2_data=be_lfemur,
        input_label="Input Motion", method1_label="Linear Euler", method2_label="Bezier Euler",
        save_path=os.path.join(OUTPUT_DIR, "graph1_le_vs_be.png")
    )

    # Graph 2: SLERP vs Bezier SLERP
    plot_comparison(
        title="Graph 2: lfemur X-rotation — SLERP vs Bezier SLERP (N=20)",
        xlabel="Frame", ylabel="Angle (degrees)",
        frame_range=frame_range_1,
        input_data=input_lfemur, method1_data=lq_lfemur, method2_data=bq_lfemur,
        input_label="Input Motion", method1_label="SLERP Quaternion", method2_label="Bezier SLERP Quaternion",
        save_path=os.path.join(OUTPUT_DIR, "graph2_lq_vs_bq.png")
    )

    # ---- Graphs 3 & 4: root, Z rotation (channel 5: tx ty tz rx ry rz), frames 200-500 ----
    joint_2 = "root"
    chan_2 = 5  # rz is the 6th value (index 5)
    fs2, fe2 = 200, 500

    input_root = parse_joint_channel(original, joint_2, chan_2, fs2, fe2)
    le_root    = parse_joint_channel(le_file,  joint_2, chan_2, fs2, fe2)
    be_root    = parse_joint_channel(be_file,  joint_2, chan_2, fs2, fe2)
    lq_root    = parse_joint_channel(lq_file,  joint_2, chan_2, fs2, fe2)
    bq_root    = parse_joint_channel(bq_file,  joint_2, chan_2, fs2, fe2)

    frame_range_2 = range(fs2, fe2 + 1)

    # Graph 3: Linear Euler vs SLERP
    plot_comparison(
        title="Graph 3: root Z-rotation — Linear Euler vs SLERP Quaternion (N=20)",
        xlabel="Frame", ylabel="Angle (degrees)",
        frame_range=frame_range_2,
        input_data=input_root, method1_data=le_root, method2_data=lq_root,
        input_label="Input Motion", method1_label="Linear Euler", method2_label="SLERP Quaternion",
        save_path=os.path.join(OUTPUT_DIR, "graph3_le_vs_lq.png")
    )

    # Graph 4: Bezier Euler vs Bezier SLERP
    plot_comparison(
        title="Graph 4: root Z-rotation — Bezier Euler vs Bezier SLERP Quaternion (N=20)",
        xlabel="Frame", ylabel="Angle (degrees)",
        frame_range=frame_range_2,
        input_data=input_root, method1_data=be_root, method2_data=bq_root,
        input_label="Input Motion", method1_label="Bezier Euler", method2_label="Bezier SLERP Quaternion",
        save_path=os.path.join(OUTPUT_DIR, "graph4_be_vs_bq.png")
    )

    print("\nAll 4 graphs generated in output/")
