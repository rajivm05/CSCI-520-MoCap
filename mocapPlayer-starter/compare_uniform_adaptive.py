"""
Compares uniform vs adaptive keyframe interpolation error in depth.
Produces: console tables + comparison plots.
"""
import os
import math
import matplotlib.pyplot as plt
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "output")

# ---- reuse the core functions from error_analysis.py ----

def euler_to_matrix(rx_deg, ry_deg, rz_deg):
    a, b, g = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cg, sg = math.cos(g), math.sin(g)
    return [
        cb*cg, sa*sb*cg - ca*sg, ca*sb*cg + sa*sg,
        cb*sg, ca*cg + sa*sb*sg, ca*sb*sg - sa*cg,
        -sb,   sa*cb,            ca*cb
    ]

def rotation_angular_distance(R1, R2):
    trace_product = 0.0
    for i in range(3):
        for k in range(3):
            trace_product += R1[k*3 + i] * R2[k*3 + i]
    cos_angle = max(-1.0, min(1.0, (trace_product - 1.0) / 2.0))
    return math.degrees(math.acos(cos_angle))

def parse_all_bone_rotations(amc_path):
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
                frames[current_frame][parts[0]] = [float(v) for v in parts[1:]]
    return frames

def get_bone_euler(bone_name, values):
    if bone_name == "root":
        return (values[3], values[4], values[5]) if len(values) >= 6 else (0, 0, 0)
    rx = values[0] if len(values) > 0 else 0.0
    ry = values[1] if len(values) > 1 else 0.0
    rz = values[2] if len(values) > 2 else 0.0
    return rx, ry, rz

def compute_per_frame_rmse(original_frames, interp_frames):
    """Returns dict: frame -> RMSE across all bones."""
    per_frame = {}
    common = sorted(set(original_frames.keys()) & set(interp_frames.keys()))
    for fr in common:
        orig_b = original_frames[fr]
        interp_b = interp_frames[fr]
        errs = []
        for bone in orig_b:
            if bone not in interp_b:
                continue
            rx1, ry1, rz1 = get_bone_euler(bone, orig_b[bone])
            rx2, ry2, rz2 = get_bone_euler(bone, interp_b[bone])
            R1 = euler_to_matrix(rx1, ry1, rz1)
            R2 = euler_to_matrix(rx2, ry2, rz2)
            errs.append(rotation_angular_distance(R1, R2))
        if errs:
            per_frame[fr] = math.sqrt(sum(e**2 for e in errs) / len(errs))
    return per_frame

def overall_rmse_from_per_frame(per_frame):
    if not per_frame:
        return 0.0
    vals = list(per_frame.values())
    return math.sqrt(sum(v**2 for v in vals) / len(vals))

def load_keyframes(path):
    with open(path) as f:
        return [int(line.strip()) for line in f if line.strip()]

# =====================================================================

if __name__ == "__main__":
    # ---------- DANCE ----------
    print("=" * 80)
    print("UNIFORM vs ADAPTIVE COMPARISON: Dance")
    print("=" * 80)

    original = parse_all_bone_rotations(os.path.join(BASE_DIR, "131_04-dance.amc"))

    dance_uniform = {
        "Linear Euler (N=20)":   os.path.join(OUTPUT_DIR, "dance-le-N20.amc"),
        "Bezier Euler (N=20)":   os.path.join(OUTPUT_DIR, "dance-be-N20.amc"),
        "SLERP Quat (N=20)":    os.path.join(OUTPUT_DIR, "dance-lq-N20.amc"),
        "Bezier SLERP (N=20)":  os.path.join(OUTPUT_DIR, "dance-bq-N20.amc"),
    }
    dance_adaptive = {
        "Linear Euler (adaptive)":   os.path.join(OUTPUT_DIR, "dance-le-adaptive.amc"),
        "Bezier Euler (adaptive)":   os.path.join(OUTPUT_DIR, "dance-be-adaptive.amc"),
        "SLERP Quat (adaptive)":     os.path.join(OUTPUT_DIR, "dance-lq-adaptive.amc"),
        "Bezier SLERP (adaptive)":   os.path.join(OUTPUT_DIR, "dance-bq-adaptive.amc"),
    }

    # keyframe gap stats
    dance_kf = load_keyframes(os.path.join(OUTPUT_DIR, "keyframes_dance_adaptive.txt"))
    dance_gaps = [dance_kf[i+1] - dance_kf[i] for i in range(len(dance_kf)-1)]
    print(f"\nAdaptive keyframes: {len(dance_kf)}, Uniform keyframes (N=20): ~{len(original)//(20+1)}")
    print(f"Adaptive gap stats: min={min(dance_gaps)}, max={max(dance_gaps)}, "
          f"mean={sum(dance_gaps)/len(dance_gaps):.1f}, median={sorted(dance_gaps)[len(dance_gaps)//2]}")
    print(f"Uniform gap: always {20+1}")

    all_dance = {**dance_uniform, **dance_adaptive}
    dance_per_frame = {}

    print(f"\n{'Method':<30} {'Overall RMSE':>14} {'Max Frame RMSE':>16} {'Worst Frame':>12}")
    print("-" * 75)

    for name, path in all_dance.items():
        interp = parse_all_bone_rotations(path)
        pf = compute_per_frame_rmse(original, interp)
        dance_per_frame[name] = pf
        rmse = overall_rmse_from_per_frame(pf)
        max_fr = max(pf, key=pf.get) if pf else -1
        max_err = pf[max_fr] if pf else 0
        print(f"{name:<30} {rmse:>13.4f}° {max_err:>15.4f}° {max_fr:>11d}")

    # ---------- MARTIAL ARTS ----------
    print(f"\n{'=' * 80}")
    print("UNIFORM vs ADAPTIVE COMPARISON: Martial Arts")
    print("=" * 80)

    original_ma = parse_all_bone_rotations(os.path.join(BASE_DIR, "135_06-martialArts.amc"))

    martial_uniform = {
        "Bezier Euler (N=40)":   os.path.join(OUTPUT_DIR, "martial-be-N40.amc"),
        "SLERP Quat (N=40)":    os.path.join(OUTPUT_DIR, "martial-lq-N40.amc"),
        "Bezier SLERP (N=40)":  os.path.join(OUTPUT_DIR, "martial-bq-N40.amc"),
    }
    martial_adaptive = {
        "Bezier Euler (adaptive)":   os.path.join(OUTPUT_DIR, "martial-be-adaptive.amc"),
        "SLERP Quat (adaptive)":     os.path.join(OUTPUT_DIR, "martial-lq-adaptive.amc"),
        "Bezier SLERP (adaptive)":   os.path.join(OUTPUT_DIR, "martial-bq-adaptive.amc"),
    }

    ma_kf = load_keyframes(os.path.join(OUTPUT_DIR, "keyframes_martial_adaptive.txt"))
    ma_gaps = [ma_kf[i+1] - ma_kf[i] for i in range(len(ma_kf)-1)]
    print(f"\nAdaptive keyframes: {len(ma_kf)}, Uniform keyframes (N=40): ~{len(original_ma)//(40+1)}")
    print(f"Adaptive gap stats: min={min(ma_gaps)}, max={max(ma_gaps)}, "
          f"mean={sum(ma_gaps)/len(ma_gaps):.1f}, median={sorted(ma_gaps)[len(ma_gaps)//2]}")
    print(f"Uniform gap: always {40+1}")

    all_martial = {**martial_uniform, **martial_adaptive}
    martial_per_frame = {}

    print(f"\n{'Method':<30} {'Overall RMSE':>14} {'Max Frame RMSE':>16} {'Worst Frame':>12}")
    print("-" * 75)

    for name, path in all_martial.items():
        interp = parse_all_bone_rotations(path)
        pf = compute_per_frame_rmse(original_ma, interp)
        martial_per_frame[name] = pf
        rmse = overall_rmse_from_per_frame(pf)
        max_fr = max(pf, key=pf.get) if pf else -1
        max_err = pf[max_fr] if pf else 0
        print(f"{name:<30} {rmse:>13.4f}° {max_err:>15.4f}° {max_fr:>11d}")

    # ===================== PLOTS =====================
    print(f"\n{'=' * 80}")
    print("GENERATING COMPARISON PLOTS...")
    print("=" * 80)

    # --- Plot 1: Per-frame error curves, uniform vs adaptive (dance, pick one method) ---
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))

    method_pairs = [
        ("Linear Euler (N=20)", "Linear Euler (adaptive)", "Linear Euler"),
        ("Bezier Euler (N=20)", "Bezier Euler (adaptive)", "Bezier Euler"),
        ("SLERP Quat (N=20)", "SLERP Quat (adaptive)", "SLERP Quaternion"),
        ("Bezier SLERP (N=20)", "Bezier SLERP (adaptive)", "Bezier SLERP Quat"),
    ]

    for ax, (uni_key, adp_key, title) in zip(axes.flat, method_pairs):
        pf_uni = dance_per_frame[uni_key]
        pf_adp = dance_per_frame[adp_key]

        frames_u = sorted(pf_uni.keys())
        frames_a = sorted(pf_adp.keys())

        ax.plot(frames_u, [pf_uni[f] for f in frames_u], 'b-', linewidth=0.8, label='Uniform (N=20)', alpha=0.8)
        ax.plot(frames_a, [pf_adp[f] for f in frames_a], 'r-', linewidth=0.8, label='Adaptive', alpha=0.8)

        # mark adaptive keyframe positions
        for kf in dance_kf:
            if kf in pf_adp:
                ax.axvline(x=kf, color='gray', alpha=0.08, linewidth=0.5)

        ax.set_title(f"Dance — {title}", fontsize=11)
        ax.set_xlabel("Frame", fontsize=9)
        ax.set_ylabel("RMSE (°)", fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)

    plt.suptitle("Per-Frame RMSE: Uniform vs Adaptive Keyframes (Dance)", fontsize=14, y=1.01)
    plt.tight_layout()
    path1 = os.path.join(OUTPUT_DIR, "compare_uniform_vs_adaptive_dance.png")
    plt.savefig(path1, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"Saved: {path1}")

    # --- Plot 2: Same for martial arts ---
    ma_method_pairs = [
        ("Bezier Euler (N=40)", "Bezier Euler (adaptive)", "Bezier Euler"),
        ("SLERP Quat (N=40)", "SLERP Quat (adaptive)", "SLERP Quaternion"),
        ("Bezier SLERP (N=40)", "Bezier SLERP (adaptive)", "Bezier SLERP Quat"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    for ax, (uni_key, adp_key, title) in zip(axes.flat, ma_method_pairs):
        pf_uni = martial_per_frame[uni_key]
        pf_adp = martial_per_frame[adp_key]

        frames_u = sorted(pf_uni.keys())
        frames_a = sorted(pf_adp.keys())

        ax.plot(frames_u, [pf_uni[f] for f in frames_u], 'b-', linewidth=0.8, label='Uniform (N=40)', alpha=0.8)
        ax.plot(frames_a, [pf_adp[f] for f in frames_a], 'r-', linewidth=0.8, label='Adaptive', alpha=0.8)

        for kf in ma_kf:
            if kf in pf_adp:
                ax.axvline(x=kf, color='gray', alpha=0.08, linewidth=0.5)

        ax.set_title(f"Martial Arts — {title}", fontsize=11)
        ax.set_xlabel("Frame", fontsize=9)
        ax.set_ylabel("RMSE (°)", fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)

    plt.suptitle("Per-Frame RMSE: Uniform vs Adaptive Keyframes (Martial Arts)", fontsize=14, y=1.01)
    plt.tight_layout()
    path2 = os.path.join(OUTPUT_DIR, "compare_uniform_vs_adaptive_martial.png")
    plt.savefig(path2, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"Saved: {path2}")

    # --- Plot 3: Bar chart comparison overall RMSE ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Dance bars
    uni_names = ["Lin Euler", "Bez Euler", "SLERP", "Bez SLERP"]
    uni_vals = [overall_rmse_from_per_frame(dance_per_frame[k]) for k in dance_uniform]
    adp_vals = [overall_rmse_from_per_frame(dance_per_frame[k]) for k in dance_adaptive]

    x = np.arange(len(uni_names))
    w = 0.35
    bars1 = axes[0].bar(x - w/2, uni_vals, w, label='Uniform (N=20)', color='#4c72b0', edgecolor='black', linewidth=0.5)
    bars2 = axes[0].bar(x + w/2, adp_vals, w, label='Adaptive (60 kf)', color='#c44e52', edgecolor='black', linewidth=0.5)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(uni_names)
    axes[0].set_ylabel("Overall RMSE (°)")
    axes[0].set_title("Dance — Uniform vs Adaptive")
    axes[0].legend()
    for bar, val in zip(bars1, uni_vals):
        axes[0].text(bar.get_x() + bar.get_width()/2, val + 0.1, f"{val:.2f}", ha='center', fontsize=8)
    for bar, val in zip(bars2, adp_vals):
        axes[0].text(bar.get_x() + bar.get_width()/2, val + 0.1, f"{val:.2f}", ha='center', fontsize=8)

    # Martial arts bars
    ma_uni_names = ["Bez Euler", "SLERP", "Bez SLERP"]
    ma_uni_vals = [overall_rmse_from_per_frame(martial_per_frame[k]) for k in martial_uniform]
    ma_adp_vals = [overall_rmse_from_per_frame(martial_per_frame[k]) for k in martial_adaptive]

    x2 = np.arange(len(ma_uni_names))
    bars3 = axes[1].bar(x2 - w/2, ma_uni_vals, w, label='Uniform (N=40)', color='#4c72b0', edgecolor='black', linewidth=0.5)
    bars4 = axes[1].bar(x2 + w/2, ma_adp_vals, w, label='Adaptive (55 kf)', color='#c44e52', edgecolor='black', linewidth=0.5)
    axes[1].set_xticks(x2)
    axes[1].set_xticklabels(ma_uni_names)
    axes[1].set_ylabel("Overall RMSE (°)")
    axes[1].set_title("Martial Arts — Uniform vs Adaptive")
    axes[1].legend()
    for bar, val in zip(bars3, ma_uni_vals):
        axes[1].text(bar.get_x() + bar.get_width()/2, val + 0.1, f"{val:.2f}", ha='center', fontsize=8)
    for bar, val in zip(bars4, ma_adp_vals):
        axes[1].text(bar.get_x() + bar.get_width()/2, val + 0.1, f"{val:.2f}", ha='center', fontsize=8)

    plt.tight_layout()
    path3 = os.path.join(OUTPUT_DIR, "compare_overall_rmse_bars.png")
    plt.savefig(path3, dpi=200)
    plt.close()
    print(f"Saved: {path3}")

    # --- Plot 4: Keyframe gap distribution histogram ---
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    axes[0].hist(dance_gaps, bins=30, color='#c44e52', edgecolor='black', linewidth=0.5)
    axes[0].axvline(x=21, color='blue', linestyle='--', linewidth=1.5, label='Uniform gap (21)')
    axes[0].set_title("Dance — Adaptive Keyframe Gap Distribution")
    axes[0].set_xlabel("Gap size (frames)")
    axes[0].set_ylabel("Count")
    axes[0].legend()

    axes[1].hist(ma_gaps, bins=30, color='#c44e52', edgecolor='black', linewidth=0.5)
    axes[1].axvline(x=41, color='blue', linestyle='--', linewidth=1.5, label='Uniform gap (41)')
    axes[1].set_title("Martial Arts — Adaptive Keyframe Gap Distribution")
    axes[1].set_xlabel("Gap size (frames)")
    axes[1].set_ylabel("Count")
    axes[1].legend()

    plt.tight_layout()
    path4 = os.path.join(OUTPUT_DIR, "compare_gap_distribution.png")
    plt.savefig(path4, dpi=200)
    plt.close()
    print(f"Saved: {path4}")

    print("\nDone.")
