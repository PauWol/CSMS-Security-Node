from lib.constants import (
    LIGHT_DAY,
    LIGHT_DUSK,
    WEIGHT_DAY,
    WEIGHT_DUSK,
    WEIGHT_NIGHT,
    THRESH_DAY,
    THRESH_DUSK,
    THRESH_NIGHT,
    PIR_BASE_SCORE,
    CONF_SCALE,
    PEAK_THRESHOLD,
    CONF_PEAK_BONUS,
    CONTINUOUS_TIME,
    BONUS_CONTINUOUS,
    STABLE_VARIANCE,
    BONUS_STABLE,
    NOISE_CUT_OUT_THRESHOLD,
)


def _get_time_weight(light: float) -> tuple[float, float, str]:
    """
    Return the time based weight-factors
    :param light: The light value as float 0 to 1 with 1 being the brightest.
    :return:
    """
    if light >= LIGHT_DAY:
        return WEIGHT_DAY, THRESH_DAY, "DAY"
    elif light >= LIGHT_DUSK:
        return WEIGHT_DUSK, THRESH_DUSK, "DUSK"
    else:
        return WEIGHT_NIGHT, THRESH_NIGHT, "NIGHT"


def compute_statistics_stable(samples):
    """
    Single-pass computation of mean, max, and variance (Welford).

    Args:
        samples: iterable of float

    Returns:
        tuple: (mean: float, max_value: float, variance: float)
    """
    n = 0
    mean = 0.0
    m2 = 0.0
    max_val = None

    for x in samples:
        if max_val is None or x > max_val:
            max_val = x

        n += 1
        delta = x - mean
        mean += delta / n
        m2 += delta * (x - mean)

    if n == 0:
        return 0.0, 0.0, 0.0

    variance = m2 / n
    return mean, max_val, variance


def calculate_score(samples, timestamps, light):
    """
    Compute a threat score from motion confidence samples.

    Combines statistical features (mean, max, variance) with heuristic bonuses:
    - Base score scaled by average confidence
    - Peak bonus if a strong signal is detected
    - Continuous motion bonus based on total time span
    - Stability bonus for low variance signals

    The final score is adjusted by a time-of-day weight (day/dusk/night),
    and compared against a phase-specific threshold.

    Args:
        samples (Iterable[float]): Confidence values (0–1).
        timestamps (Sequence[float]): Corresponding timestamps (monotonic).
        light (float): Ambient light level (0–1).

    Returns:
        tuple:
            score (float): Final weighted threat score.
            threshold (float): Phase-dependent trigger threshold.
            phase (str): One of {"DAY", "DUSK", "NIGHT"}.

    Note:
        Duration is computed as timestamps[-1] - timestamps[0] and does not
        account for gaps between samples.
    """
    weight, threshold, phase = _get_time_weight(light)

    score = PIR_BASE_SCORE

    if len(samples) == 0:
        return 0, threshold, phase

    avg_conf, max_conf, variance = compute_statistics_stable(samples)

    if avg_conf < NOISE_CUT_OUT_THRESHOLD:
        return 0, threshold, phase

    # radar core
    score += avg_conf * CONF_SCALE

    # strong peak
    if max_conf >= PEAK_THRESHOLD:
        score += CONF_PEAK_BONUS

    # continuous motion
    duration = timestamps[-1] - timestamps[0]
    if duration >= CONTINUOUS_TIME and avg_conf > 0.3:
        score += BONUS_CONTINUOUS

    # stability
    if variance < STABLE_VARIANCE and avg_conf > 0.25:
        score += BONUS_STABLE

    score = score * weight

    return score, threshold, phase
