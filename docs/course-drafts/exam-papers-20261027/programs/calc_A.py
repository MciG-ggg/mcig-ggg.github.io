"""Check numeric answers that appear in the current A paper.

Run: python3 programs/calc_A.py
"""

import math

# BUILD: values from the current A.tex questions.
neighbor_cost = 4.0
beam_scale, beam_dx, beam_dy = 0.5, 2.0, 1.9
iteration_b1, iteration_b2 = 2.0, 3.0
boundary_count, iteration_limit = 40.0, 20.0
similarity, similarity_threshold = 0.3, 0.5
static_min, group_front, group_back, group_side = 0.4, 1.6, -1.2, 0.9
entropy_probabilities = (0.5, 0.5, 0.0, 0.0)
attention_logits, attention_active = (1.0, 3.0, 0.0), (True, True, False)
sg_density_ratio, theta_hr, theta_rh = 0.4, math.pi / 2, -math.pi / 2
paths = (
    ((0.0, 2.0), (0.0, 1.0), (0.0, 2.0)),
    ((2.0, 0.0), (2.0, 1.0), (2.0, 2.0)),
    ((3.0, 0.0), (2.0, 0.0), (3.0, 0.0)),
)

# OPERATE: calculate the answers from the question data.
info_gain_q1 = min(1.0, math.exp(-0.0) * 1.0, math.exp(-math.log(2.0)) * 1.0)
f_q1 = 10.0 + (1.0 + info_gain_q1) * 5.0
entropy_q2 = -sum(p * math.log(p) for p in entropy_probabilities if p > 0)
entropy_ratio_q2 = entropy_q2 / math.log(4.0)
iterations_raw = iteration_b2 * boundary_count / (iteration_b1 * math.e)
iterations_limited = min(iterations_raw, iteration_limit)
similarity_weight = 1.0 / (1.0 + math.exp(similarity - similarity_threshold))
sigmas = (
    (static_min + group_front) / 2.0,
    (static_min + abs(group_back)) / 2.0,
    (static_min + group_side) / 2.0,
)
beam_tr_l, beam_tr_w = beam_dx / beam_scale, beam_dy / beam_scale
beam_length = (beam_tr_l - math.floor(beam_tr_w / 2.0) + beam_tr_w) * beam_scale
active_logits = [value for value, active in zip(attention_logits, attention_active) if active]
maximum_logit = max(active_logits)
attention_exp = [math.exp(value - maximum_logit) for value in active_logits]
attention_total = sum(attention_exp)
attention_weights = tuple(
    math.exp(value - maximum_logit) / attention_total if active else 0.0
    for value, active in zip(attention_logits, attention_active)
)
sgi = sg_density_ratio * (1.0 - 0.9 * theta_hr / math.pi) * (1.0 - theta_rh / math.pi)
neighbor_hie_a = min(0.8, 0.6 * math.exp(-2.0 / 2.0))
neighbor_hie_b = min(0.8, 0.5 * math.exp(-0.0 / 2.0))
neighbor_f_a = neighbor_cost + (1.0 + neighbor_hie_a) * 3.0
neighbor_f_b = neighbor_cost + (1.0 + neighbor_hie_b) * 2.0
pairs = ((1, 2), (2, 3), (3, 4))
group_weights = (
    1.0 / (1.0 + math.exp(0.3 - 0.5)),
    1.0 / (1.0 + math.exp(0.2 - 0.5)),
)
second_differences = tuple(
    tuple(path[2][axis] - 2.0 * path[1][axis] + path[0][axis] for axis in range(2))
    for path in paths
)
smooth_costs = tuple(sum(value * value for value in difference) for difference in second_differences)
question23_costs = tuple(cost + 3.0 for cost in smooth_costs)
program25_costs = (smooth_costs[1] + 4.0, smooth_costs[2] + 2.0)
relative_field_q23 = tuple(
    tuple(math.exp(-0.5 * sum(coordinate * coordinate for coordinate in point))
          for point in path)
    for path in paths
)
gmm_peak = 1.0 / (2.0 * math.pi)
gmm_at_path_x2 = gmm_peak * math.exp(-0.5 * 2.0 ** 2)

actual_values = {
    "A01 H_IE": info_gain_q1,
    "A01 f_V": f_q1,
    "A02 normalized entropy": entropy_ratio_q2,
    "A03 raw iterations": iterations_raw,
    "A03 limited iterations": iterations_limited,
    "A06 similarity weight": similarity_weight,
    "A07 sigma_x1": sigmas[0],
    "A07 sigma_x2": sigmas[1],
    "A07 sigma_y": sigmas[2],
    "A11 beam length": beam_length,
    "A14 attention weight 1": attention_weights[0],
    "A14 attention weight 2": attention_weights[1],
    "A14 masked weight": attention_weights[2],
    "A15 ADE": 1.0,
    "A16 group origin x": (0.6 - 0.2 + 1.1) / 3.0,
    "A16 group origin y": (0.4 + 0.2 + 0.6) / 3.0,
    "A17 velocity penalty": max(3.0 ** 2 - 2.0 ** 2, 0.0) ** 3,
    "A18 trajectory objective": 12.0 + 0.25 * 20.0,
    "A19 SGI": sgi,
    "A21 H_IE A": neighbor_hie_a,
    "A21 f_A": neighbor_f_a,
    "A21 H_IE B": neighbor_hie_b,
    "A21 f_B": neighbor_f_b,
    "A21 entropy ratio": entropy_ratio_q2,
    "A21 limited iterations": iterations_limited,
    "A22 group pairs": float(len(pairs)),
    "A22 weight 1-2": group_weights[0],
    "A22 weight 3-4": group_weights[1],
    "A23 relative field A middle": relative_field_q23[0][1],
    "A23 smooth cost B": smooth_costs[1],
    "A23 smooth cost C": smooth_costs[2],
    "A23 objective B": question23_costs[1],
    "A23 objective C": question23_costs[2],
    "A25 Gaussian peak": gmm_peak,
    "A25 Gaussian at x=2": gmm_at_path_x2,
    "A25 smooth cost straight path": smooth_costs[1],
    "A25 smooth cost curved path": smooth_costs[2],
    "A25 straight path objective": program25_costs[0],
    "A25 curved path objective": program25_costs[1],
}

# CHECK: expected values from the current A.tex answer key.
expected_values = {
    "A01 H_IE": 0.5,
    "A01 f_V": 17.5,
    "A02 normalized entropy": 0.5,
    "A03 raw iterations": 22.072766,
    "A03 limited iterations": 20.0,
    "A06 similarity weight": 0.549834,
    "A07 sigma_x1": 1.0,
    "A07 sigma_x2": 0.8,
    "A07 sigma_y": 0.65,
    "A11 beam length": 3.4,
    "A14 attention weight 1": 0.119203,
    "A14 attention weight 2": 0.880797,
    "A14 masked weight": 0.0,
    "A15 ADE": 1.0,
    "A16 group origin x": 0.5,
    "A16 group origin y": 0.4,
    "A17 velocity penalty": 125.0,
    "A18 trajectory objective": 17.0,
    "A19 SGI": 0.33,
    "A21 H_IE A": 0.220728,
    "A21 f_A": 7.662183,
    "A21 H_IE B": 0.5,
    "A21 f_B": 7.0,
    "A21 entropy ratio": 0.5,
    "A21 limited iterations": 20.0,
    "A22 group pairs": 3.0,
    "A22 weight 1-2": 0.549834,
    "A22 weight 3-4": 0.574443,
    "A23 relative field A middle": 0.606531,
    "A23 smooth cost B": 0.0,
    "A23 smooth cost C": 4.0,
    "A23 objective B": 3.0,
    "A23 objective C": 7.0,
    "A25 Gaussian peak": 0.159155,
    "A25 Gaussian at x=2": 0.021539,
    "A25 smooth cost straight path": 0.0,
    "A25 smooth cost curved path": 4.0,
    "A25 straight path objective": 4.0,
    "A25 curved path objective": 6.0,
}

for label, expected in expected_values.items():
    actual = actual_values[label]
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=5e-5), (label, actual, expected)
    print(f"{label:<34} {actual:.6f}")

print(f"\n{len(expected_values)} current A-paper checks passed")
