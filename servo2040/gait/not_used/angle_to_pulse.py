from typing import Dict, Tuple

# Asymmetric calibration data (m+ and m- in µs/°)
CALIBRATION: Dict[str, Tuple[float, float]] = {
    "j11": (10.6667, 10.8889),
    "j12": (10.4444, 11.7778),
    "j13": (10.4444, 11.3333),
    "j21": (10.3333, 11.3333),
    "j22": (10.5556, 12.0000),
    "j23": (11.1667, 11.1667),
    "j31": (11.1111, 10.6667),
    "j32": (11.1111, 10.8889),
    "j33": (11.0000, 10.6667),
    "j41": (10.5556, 10.8889),
    "j42": (11.1111, 11.0000),
    "j43": (10.7778, 11.1111),
    "j51": (12.0000, 11.1111),
    "j52": (10.6667, 11.1111),
    "j53": (10.6667, 10.8889),
    "j61": (10.8889, 11.1111),
    "j62": (11.5556, 10.4444),
    "j63": (11.0000, 10.5556),
}

CENTER_PULSE = 1500.0  # µs


def angle_to_pulse(joint: str, theta: float) -> float:
    """Convert angle (degrees) to pulse (µs) using asymmetric calibration."""
    m_plus, m_minus = CALIBRATION[joint]
    if theta >= 0:
        return CENTER_PULSE + m_plus * theta
    else:
        return CENTER_PULSE + m_minus * theta  # theta is negative

if __name__ == "__main__":

    joint = "j12"
    angle = 45
    pulse = angle_to_pulse(joint, angle)
    print(f"{joint}: {angle} → {pulse:.0f} µs")
