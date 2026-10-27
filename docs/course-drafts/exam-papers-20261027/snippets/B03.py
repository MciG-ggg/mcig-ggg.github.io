episodes = [250, 1000, 1200]
lambda_min = 0.6
c = 1000
outcomes = [
    (0.5, 0.751, 0.218, 5.815),
    (0.6, 0.816, 0.153, 6.356),
    (0.7, 0.859, 0.118, 6.506),
    (0.8, 0.910, 0.067, 6.727),
    (0.9, 0.876, 0.091, 6.659),
]
result = analyze_bichannel(episodes, lambda_min, c, outcomes)
