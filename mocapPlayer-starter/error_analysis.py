import os
import math
import matplotlib.pyplot as plt
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "output")

def euler_to_matrix(rx_deg, ry_deg, rz_deg):
    """Convert XYZ euler angles (degrees) to a 3x3 rotation matrix (Rz*Ry*Rx)."""
    a = math.radians(rx_deg)
    b = math.radians(ry_deg)
    g = math.radians(rz_deg)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cg, sg = math.cos(g), math.sin(g)
    return [
        cb*cg,          sa*sb*cg - ca*sg,  ca*sb*cg + sa*sg,
        cb*sg,          ca*cg + sa*sb*sg,  ca*sb*sg - sa*cg,
        -sb,            sa*cb,             ca*cb
    ]

def rotation_angular_distance(R1, R2):
    """
    Computes the angular distance (in degrees) between two rotation matrices.
    Uses: angle = arccos((trace(R1^T * R2) - 1) / 2)
    """
    # compute R1^T * R2
    trace_val = 0.0
    for i in range(3):
        for j in range(3):
            # R1^T[i][j] = R1[j][i], so (R1^T * R2)[i][i] = sum_k R1[k][i]*R2[k][i]
            trace_val += R1[i*3 + j] * R2[i*3 + j]  # this is actually trace(R1^T R2) shortcut

    # wait, let me be more careful
    # trace(R1^T * R2) = sum_i sum_j R1[j][i] * R2[j][i] = sum of element-wise products
    # which is what I computed above (Frobenius inner product)
    # Actually no. Let me recompute properly.

    # R1^T * R2, then take trace
    # (R1^T * R2)[i][i] = sum_k R1^T[i][k] * R2[k][i] = sum_k R1[k][i] * R2[k][i]
    trace_product = 0.0
    for i in range(3):
        for k in range(3):
            trace_product += R1[k*3 + i] * R2[k*3 + i]

    # clamp to valid range for acos
    cos_angle = (trace_product - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def parse_all_bone_rotations(amc_path):
    """Parses an AMC file. Returns: frame_number -> { bone_name: [values...] }"""
    frames = {}
    current_frame = None
    with open(amc_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith(':'):
                continue
            parts = line.split()
            if len(parts) == 1:
                try:
                    current_frame = int(parts[0])
                    frames[current_frame] = {}
                    continue
                except ValueError:
                    pass
            if current_frame is not None and len(parts) >= 2:
                bone_name = parts[0]
                values = [float(v) for v in parts[1:]]
                frames[current_frame][bone_name] = values
    return frames


def get_bone_euler(bone_name, values):
    """Extracts the rotation euler angles (rx, ry, rz) for a bone.
    Root has tx,ty,tz,rx,ry,rz; other bones just have rotation channels."""
    if bone_name == "root":
        if len(values) >= 6:
            return values[3], values[4], values[5]
        else:
            return 0.0, 0.0, 0.0
    else:
        # pad to 3 if fewer channels
        rx = values[0] if len(values) > 0 else 0.0
        ry = values[1] if len(values) > 1 else 0.0
        rz = values[2] if len(values) > 2 else 0.0
        return rx, ry, rz


def compute_rotation_error(original_frames, interp_frames):
    """
    Computes per-frame rotation error using proper angular distance between
    rotation matrices (representation-independent, no euler wrapping issues).
    Returns: (per_frame_avg_error, overall_rmse, max_error, max_error_frame)
    """
    sum_sq_error = 0.0
    total_count = 0
    max_error = 0.0
    max_error_frame = -1
    per_frame_error = {}

    common_frames = sorted(set(original_frames.keys()) & set(interp_frames.keys()))

    for fr in common_frames:
        orig_bones = original_frames[fr]
        interp_bones = interp_frames[fr]
        frame_errors = []

        for bone_name in orig_bones:
            if bone_name not in interp_bones:
                continue

            rx1, ry1, rz1 = get_bone_euler(bone_name, orig_bones[bone_name])
            rx2, ry2, rz2 = get_bone_euler(bone_name, interp_bones[bone_name])

            R1 = euler_to_matrix(rx1, ry1, rz1)
            R2 = euler_to_matrix(rx2, ry2, rz2)

            ang_dist = rotation_angular_distance(R1, R2)
            frame_errors.append(ang_dist)

        if frame_errors:
            frame_avg = sum(e**2 for e in frame_errors) / len(frame_errors)
            frame_rmse = math.sqrt(frame_avg)
            per_frame_error[fr] = frame_rmse
            sum_sq_error += sum(e**2 for e in frame_errors)
            total_count += len(frame_errors)

            if frame_rmse > max_error:
                max_error = frame_rmse
                max_error_frame = fr

    overall_rmse = math.sqrt(sum_sq_error / total_count) if total_count > 0 else 0.0
    return per_frame_error, overall_rmse, max_error, max_error_frame


def compute_per_bone_error(original_frames, interp_frames):
    """Computes RMSE per bone using angular distance. Returns: bone_name -> rmse."""
    bone_sq_sums = {}
    bone_counts = {}

    common_frames = sorted(set(original_frames.keys()) & set(interp_frames.keys()))

    for fr in common_frames:
        orig_bones = original_frames[fr]
        interp_bones = interp_frames[fr]

        for bone_name in orig_bones:
            if bone_name not in interp_bones:
                continue

            rx1, ry1, rz1 = get_bone_euler(bone_name, orig_bones[bone_name])
            rx2, ry2, rz2 = get_bone_euler(bone_name, interp_bones[bone_name])

            R1 = euler_to_matrix(rx1, ry1, rz1)
            R2 = euler_to_matrix(rx2, ry2, rz2)
            ang_dist = rotation_angular_distance(R1, R2)

            if bone_name not in bone_sq_sums:
                bone_sq_sums[bone_name] = 0.0
                bone_counts[bone_name] = 0
            bone_sq_sums[bone_name] += ang_dist ** 2
            bone_counts[bone_name] += 1

    return {b: math.sqrt(bone_sq_sums[b] / bone_counts[b]) for b in bone_sq_sums}


def plot_overall_rmse_bars(dance_results, martial_results):
    """Plot 1: Bar chart comparing overall RMSE across methods."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Dance
    names_d = list(dance_results.keys())
    vals_d = [dance_results[n] for n in names_d]
    colors_d = ['#4c72b0', '#55a868', '#c44e52', '#8172b2']
    bars = axes[0].bar(names_d, vals_d, color=colors_d, edgecolor='black', linewidth=0.6)
    axes[0].set_title("Dance (N=20) — Overall RMSE", fontsize=13)
    axes[0].set_ylabel("RMSE (degrees)", fontsize=11)
    axes[0].set_ylim(0, max(vals_d) * 1.25)
    for bar, val in zip(bars, vals_d):
        axes[0].text(bar.get_x() + bar.get_width() / 2, val + 0.15,
                     f"{val:.2f}°", ha='center', fontsize=9, fontweight='bold')
    axes[0].tick_params(axis='x', rotation=15)

    # Martial Arts
    names_m = list(martial_results.keys())
    vals_m = [martial_results[n] for n in names_m]
    colors_m = ['#55a868', '#c44e52', '#8172b2']
    bars = axes[1].bar(names_m, vals_m, color=colors_m, edgecolor='black', linewidth=0.6)
    axes[1].set_title("Martial Arts (N=40) — Overall RMSE", fontsize=13)
    axes[1].set_ylabel("RMSE (degrees)", fontsize=11)
    axes[1].set_ylim(0, max(vals_m) * 1.25)
    for bar, val in zip(bars, vals_m):
        axes[1].text(bar.get_x() + bar.get_width() / 2, val + 0.15,
                     f"{val:.2f}°", ha='center', fontsize=9, fontweight='bold')
    axes[1].tick_params(axis='x', rotation=15)

    plt.tight_layout()
    save_path = os.path.join(OUTPUT_DIR, "error_overall_rmse.png")
    plt.savefig(save_path, dpi=200)
    plt.close()
    print(f"Saved: {save_path}")


def plot_per_bone_bars(dance_bone_data, martial_bone_data):
    """Plot 2: Grouped bar chart of top-10 worst bones per method."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    method_colors = {
        "Linear Euler": '#4c72b0',
        "Bezier Euler": '#55a868',
        "SLERP Quaternion": '#c44e52',
        "Bezier SLERP Quat": '#8172b2',
    }

    for ax, bone_data, title in [
        (axes[0], dance_bone_data, "Dance (N=20) — Per-Bone RMSE (Top 10)"),
        (axes[1], martial_bone_data, "Martial Arts (N=40) — Per-Bone RMSE (Top 10)")
    ]:
        # collect union of top-10 bones across all methods
        all_top_bones = set()
        for method_name, bone_rmse in bone_data.items():
            top = sorted(bone_rmse.items(), key=lambda x: x[1], reverse=True)[:10]
            for bone, _ in top:
                all_top_bones.add(bone)

        # sort bones by max RMSE across methods
        bone_max = {}
        for bone in all_top_bones:
            bone_max[bone] = max(bone_data[m].get(bone, 0) for m in bone_data)
        sorted_bones = sorted(bone_max.keys(), key=lambda b: bone_max[b], reverse=True)[:12]

        x = np.arange(len(sorted_bones))
        n_methods = len(bone_data)
        width = 0.8 / n_methods

        for idx, (method_name, bone_rmse) in enumerate(bone_data.items()):
            vals = [bone_rmse.get(b, 0) for b in sorted_bones]
            ax.bar(x + idx * width - 0.4 + width / 2, vals, width,
                   label=method_name, color=method_colors.get(method_name, '#999'),
                   edgecolor='black', linewidth=0.4)

        ax.set_title(title, fontsize=13)
        ax.set_ylabel("RMSE (degrees)", fontsize=11)
        ax.set_xticks(x)
        ax.set_xticklabels(sorted_bones, rotation=30, ha='right', fontsize=9)
        ax.legend(fontsize=9)
        ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    save_path = os.path.join(OUTPUT_DIR, "error_per_bone.png")
    plt.savefig(save_path, dpi=200)
    plt.close()
    print(f"Saved: {save_path}")


def plot_per_frame_curves(dance_frame_data, martial_frame_data):
    """Plot 3: Per-frame RMSE over time for each method."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    method_styles = {
        "Linear Euler":       ('b-',  1.0),
        "Bezier Euler":       ('g--', 1.0),
        "SLERP Quaternion":   ('r-.', 1.0),
        "Bezier SLERP Quat": ('m:',  1.2),
    }

    for ax, frame_data, title in [
        (axes[0], dance_frame_data, "Dance (N=20) — Per-Frame RMSE Over Time"),
        (axes[1], martial_frame_data, "Martial Arts (N=40) — Per-Frame RMSE Over Time")
    ]:
        for method_name, per_frame in frame_data.items():
            frames_sorted = sorted(per_frame.keys())
            vals = [per_frame[fr] for fr in frames_sorted]
            style, lw = method_styles.get(method_name, ('k-', 1.0))
            ax.plot(frames_sorted, vals, style, linewidth=lw, label=method_name, alpha=0.8)

        ax.set_title(title, fontsize=13)
        ax.set_xlabel("Frame", fontsize=11)
        ax.set_ylabel("RMSE (degrees)", fontsize=11)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = os.path.join(OUTPUT_DIR, "error_per_frame.png")
    plt.savefig(save_path, dpi=200)
    plt.close()
    print(f"Saved: {save_path}")


if __name__ == "__main__":
    # ======================== DANCE N=20 ========================
    print("=" * 70)
    print("ERROR ANALYSIS: 131_04-dance.amc, N=20")
    print("(using rotation matrix angular distance — representation-independent)")
    print("=" * 70)

    original = parse_all_bone_rotations(os.path.join(BASE_DIR, "131_04-dance.amc"))

    methods = {
        "Linear Euler":         os.path.join(OUTPUT_DIR, "dance-le-N20.amc"),
        "Bezier Euler":         os.path.join(OUTPUT_DIR, "dance-be-N20.amc"),
        "SLERP Quaternion":     os.path.join(OUTPUT_DIR, "dance-lq-N20.amc"),
        "Bezier SLERP Quat":   os.path.join(OUTPUT_DIR, "dance-bq-N20.amc"),
    }

    print(f"\n{'Method':<25} {'Overall RMSE':>14} {'Max Frame RMSE':>16} {'Worst Frame':>12}")
    print("-" * 70)

    dance_overall = {}
    dance_frame_data = {}
    dance_bone_data = {}

    for method_name, filepath in methods.items():
        interp = parse_all_bone_rotations(filepath)
        per_frame, overall, max_err, max_fr = compute_rotation_error(original, interp)
        dance_overall[method_name] = overall
        dance_frame_data[method_name] = per_frame
        dance_bone_data[method_name] = compute_per_bone_error(original, interp)
        print(f"{method_name:<25} {overall:>14.4f}° {max_err:>15.4f}° {max_fr:>11d}")

    # per-bone breakdown
    print(f"\n{'':=<70}")
    print("PER-BONE RMSE (top 10 worst bones per method)")
    print(f"{'':=<70}")

    for method_name in methods:
        sorted_bones = sorted(dance_bone_data[method_name].items(), key=lambda x: x[1], reverse=True)
        print(f"\n  {method_name}:")
        for bone, rmse in sorted_bones[:10]:
            print(f"    {bone:<20} {rmse:.4f}°")

    # ======================== MARTIAL ARTS N=40 ========================
    print(f"\n{'=' * 70}")
    print("ERROR ANALYSIS: 135_06-martialArts.amc, N=40")
    print("(using rotation matrix angular distance — representation-independent)")
    print("=" * 70)

    original_ma = parse_all_bone_rotations(os.path.join(BASE_DIR, "135_06-martialArts.amc"))

    methods_ma = {
        "Bezier Euler":         os.path.join(OUTPUT_DIR, "martial-be-N40.amc"),
        "SLERP Quaternion":     os.path.join(OUTPUT_DIR, "martial-lq-N40.amc"),
        "Bezier SLERP Quat":   os.path.join(OUTPUT_DIR, "martial-bq-N40.amc"),
    }

    print(f"\n{'Method':<25} {'Overall RMSE':>14} {'Max Frame RMSE':>16} {'Worst Frame':>12}")
    print("-" * 70)

    martial_overall = {}
    martial_frame_data = {}
    martial_bone_data = {}

    for method_name, filepath in methods_ma.items():
        interp = parse_all_bone_rotations(filepath)
        per_frame, overall, max_err, max_fr = compute_rotation_error(original_ma, interp)
        martial_overall[method_name] = overall
        martial_frame_data[method_name] = per_frame
        martial_bone_data[method_name] = compute_per_bone_error(original_ma, interp)
        print(f"{method_name:<25} {overall:>14.4f}° {max_err:>15.4f}° {max_fr:>11d}")

    print(f"\n{'':=<70}")
    print("PER-BONE RMSE (top 10 worst bones per method)")
    print(f"{'':=<70}")

    for method_name in methods_ma:
        sorted_bones = sorted(martial_bone_data[method_name].items(), key=lambda x: x[1], reverse=True)
        print(f"\n  {method_name}:")
        for bone, rmse in sorted_bones[:10]:
            print(f"    {bone:<20} {rmse:.4f}°")

    # ======================== GENERATE PLOTS ========================
    print(f"\n{'=' * 70}")
    print("GENERATING PLOTS...")
    print(f"{'=' * 70}")

    plot_overall_rmse_bars(dance_overall, martial_overall)
    plot_per_bone_bars(dance_bone_data, martial_bone_data)
    plot_per_frame_curves(dance_frame_data, martial_frame_data)

    print("\nDone.")
