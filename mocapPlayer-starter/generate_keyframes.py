"""
Adaptive keyframe selection for non-uniform interpolation (extra credit).

Analyzes an AMC motion file and selects keyframes at irregular intervals based
on motion complexity: frames with rapid angular changes get denser keyframes,
while smooth/slow sections get sparser ones.

Usage:
  python3 generate_keyframes.py <amc_file> <num_keyframes> <output_keyframe_file>

Example:
  python3 generate_keyframes.py 131_04-dance.amc 60 keyframes_dance_adaptive.txt
"""

import sys
import os
import math

def parse_bone_rotations(amc_path):
    """Parses an AMC file. Returns list of (frame_number, {bone: [vals]}) in order."""
    frames = []
    current_frame = None
    current_bones = {}
    with open(amc_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith(':'):
                continue
            parts = line.split()
            if len(parts) == 1:
                try:
                    if current_frame is not None:
                        frames.append((current_frame, current_bones))
                    current_frame = int(parts[0])
                    current_bones = {}
                    continue
                except ValueError:
                    pass
            if current_frame is not None and len(parts) >= 2:
                bone_name = parts[0]
                values = [float(v) for v in parts[1:]]
                current_bones[bone_name] = values
    if current_frame is not None:
        frames.append((current_frame, current_bones))
    return frames


def compute_frame_importance(frames):
    """
    Computes an importance score for each frame based on angular velocity.
    Higher score = more change from previous frame = more important to sample.
    Uses the sum of squared angle differences across all bone channels.
    """
    scores = [0.0] * len(frames)

    for i in range(1, len(frames)):
        _, prev_bones = frames[i - 1]
        _, curr_bones = frames[i]
        total_delta = 0.0

        for bone in curr_bones:
            if bone not in prev_bones:
                continue
            prev_vals = prev_bones[bone]
            curr_vals = curr_bones[bone]
            # skip translation channels for root (first 3 values)
            rot_start = 3 if bone == "root" else 0
            for ch in range(rot_start, min(len(prev_vals), len(curr_vals))):
                diff = curr_vals[ch] - prev_vals[ch]
                total_delta += diff * diff

        scores[i] = math.sqrt(total_delta)

    return scores


def select_adaptive_keyframes(frames, scores, num_keyframes):
    """
    Greedy selection: always include first and last frame.
    Then iteratively pick the frame with the highest cumulative importance
    in the largest gap between existing keyframes.

    This ensures dense sampling where motion is complex and sparse where it's smooth.
    """
    n = len(frames)
    if num_keyframes >= n:
        return list(range(n))
    if num_keyframes < 2:
        return [0]

    # build cumulative importance for efficient range queries
    cumulative = [0.0] * (n + 1)
    for i in range(n):
        cumulative[i + 1] = cumulative[i] + scores[i]

    def range_importance(lo, hi):
        """Total importance of frames in the open interval (lo, hi)."""
        return cumulative[hi] - cumulative[lo + 1]

    # start with first and last frame as keyframes
    selected = {0, n - 1}

    # use a priority queue approach: for each gap between selected keyframes,
    # find the frame with highest score and track the gap's total importance
    import heapq

    # gaps stored as (-importance, left_keyframe, right_keyframe)
    # negative because heapq is a min-heap and we want max importance first
    sorted_sel = sorted(selected)
    heap = []
    for j in range(len(sorted_sel) - 1):
        lo, hi = sorted_sel[j], sorted_sel[j + 1]
        if hi - lo > 1:
            imp = range_importance(lo, hi)
            heapq.heappush(heap, (-imp, lo, hi))

    while len(selected) < num_keyframes and heap:
        neg_imp, lo, hi = heapq.heappop(heap)

        # verify this gap is still valid (both endpoints still adjacent in selected set)
        if lo not in selected or hi not in selected:
            continue
        if hi - lo <= 1:
            continue

        # find the frame with highest individual score in this gap
        best_frame = lo + 1
        best_score = scores[lo + 1]
        for f in range(lo + 2, hi):
            if scores[f] > best_score:
                best_score = scores[f]
                best_frame = f

        selected.add(best_frame)

        # split the gap into two sub-gaps and push them
        if best_frame - lo > 1:
            imp_left = range_importance(lo, best_frame)
            heapq.heappush(heap, (-imp_left, lo, best_frame))
        if hi - best_frame > 1:
            imp_right = range_importance(best_frame, hi)
            heapq.heappush(heap, (-imp_right, best_frame, hi))

    return sorted(selected)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 generate_keyframes.py <amc_file> <num_keyframes> <output_file>")
        sys.exit(1)

    amc_path = sys.argv[1]
    num_kf = int(sys.argv[2])
    output_path = sys.argv[3]

    print(f"Parsing {amc_path}...")
    frames = parse_bone_rotations(amc_path)
    print(f"  Total frames: {len(frames)}")

    print("Computing per-frame importance scores...")
    scores = compute_frame_importance(frames)

    print(f"Selecting {num_kf} adaptive keyframes...")
    keyframe_indices = select_adaptive_keyframes(frames, scores, num_kf)

    # the AMC parser returns 1-based frame numbers, but our interpolator uses 0-based indexing
    # convert: frame index in the motion array = frame_number - 1 (since AMC frames start at 1)
    first_frame_num = frames[0][0]  # typically 1
    zero_based = [i for i in keyframe_indices]  # already 0-based array indices

    with open(output_path, 'w') as f:
        for idx in zero_based:
            f.write(f"{idx}\n")

    print(f"Wrote {len(zero_based)} keyframe indices to {output_path}")

    # print some statistics about the spacing
    gaps = [zero_based[i+1] - zero_based[i] for i in range(len(zero_based)-1)]
    if gaps:
        print(f"  Min gap: {min(gaps)} frames")
        print(f"  Max gap: {max(gaps)} frames")
        print(f"  Mean gap: {sum(gaps)/len(gaps):.1f} frames")
        print(f"  Equivalent uniform N would be: ~{len(frames) // num_kf}")
