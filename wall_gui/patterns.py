ROWS = 3
COLS = 3

# Grid bit indices:
# 0 1 2
# 3 4 5
# 6 7 8

def cell_to_bit(r, c):
    return r * COLS + c

def make_mask(cells):
    mask = 0
    for r, c in cells:
        if 0 <= r < ROWS and 0 <= c < COLS:
            bit = cell_to_bit(r, c)
            mask |= (1 << bit)
    return mask

def all_off():
    return 0

def all_on():
    return 511

def pattern_wave_lr():
    return [
        make_mask([(0, 0), (1, 0), (2, 0)]),  # left col
        make_mask([(0, 1), (1, 1), (2, 1)]),  # middle col
        make_mask([(0, 2), (1, 2), (2, 2)]),  # right col
        make_mask([(0, 1), (1, 1), (2, 1)]),  # back through middle
    ]

def pattern_wave_tb():
    return [
        make_mask([(0, 0), (0, 1), (0, 2)]),  # top row
        make_mask([(1, 0), (1, 1), (1, 2)]),  # middle row
        make_mask([(2, 0), (2, 1), (2, 2)]),  # bottom row
        make_mask([(1, 0), (1, 1), (1, 2)]),  # back through middle
    ]

def pattern_ripple():
    center = make_mask([(1, 1)])
    cross = make_mask([(1, 1), (0, 1), (1, 0), (1, 2), (2, 1)])
    outer_ring = make_mask([
        (0, 0), (0, 1), (0, 2),
        (1, 0),         (1, 2),
        (2, 0), (2, 1), (2, 2)
    ])
    full = all_on()

    return [
        center,
        cross,
        outer_ring,
        full,
        outer_ring,
        cross,
        center,
        all_off(),
    ]

def pattern_checker():
    a = make_mask([(0, 0), (0, 2), (1, 1), (2, 0), (2, 2)])
    b = make_mask([(0, 1), (1, 0), (1, 2), (2, 1)])
    return [a, b]

def pattern_snake():
    path = [
        (0, 0), (0, 1), (0, 2),
        (1, 2), (1, 1), (1, 0),
        (2, 0), (2, 1), (2, 2),
    ]
    return [make_mask([cell]) for cell in path]

def pattern_pulse():
    return [all_on(), all_off()]

PATTERN_MAP = {
    "wave_lr": pattern_wave_lr,
    "wave_tb": pattern_wave_tb,
    "ripple": pattern_ripple,
    "checker": pattern_checker,
    "snake": pattern_snake,
    "pulse": pattern_pulse,
}

def get_pattern_names():
    return list(PATTERN_MAP.keys())

def get_pattern_frames(name):
    if name not in PATTERN_MAP:
        raise ValueError(f"Unknown pattern: {name}")
    return PATTERN_MAP[name]()

def get_demo_sequence():
    """
    Returns a list of tuples:
    (pattern_name, loops, frame_delay)
    """
    return [
        ("ripple", 3, 0.40),
        ("wave_lr", 4, 0.30),
        ("wave_tb", 4, 0.30),
        ("checker", 6, 0.35),
        ("pulse", 4, 0.40),
        ("snake", 2, 0.20),
    ]