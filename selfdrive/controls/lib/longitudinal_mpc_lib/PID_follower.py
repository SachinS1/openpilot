import csv
import numpy as np


class VelocityProfilePID:
  """
  Ultra-simple PID speed controller.
  - Fixed dt = 0.05 s (20 Hz)
  - CSV format (always): header 'timestep,speed' then rows of time[s], speed[m/s]
  - Each call to step() advances one sample.
  - Output: desired acceleration [m/s^2]
  """

  def __init__(self, csv_path: str, kp: float = 0.6, ki: float = 0.2, kd: float = 0.05,
               dt: float = 0.05, accel_min: float = -5.0, accel_max: float = 5.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.accel_min = accel_min
    self.accel_max = accel_max

    # Load CSV (header always present: timestep,speed)
    t_list, v_list = [], []
    with open(csv_path, "r", newline="") as f:
      reader = csv.reader(f)
      next(reader)  # skip header
      for r in reader:
        if len(r) >= 2:
          t_list.append(float(r[0]))
          v_list.append(float(r[1]))

    # Store as arrays
    self.t = np.array(t_list, dtype=float)
    self.v = np.array(v_list, dtype=float)
    self.n = len(self.v)

    # PID state
    self.k = 0  # sample index
    self.int_e = 0.0
    self.prev_e = 0.0
    self._first = True

  def reset(self):
    self.k = 0
    self.int_e = 0.0
    self.prev_e = 0.0
    self._first = True

  def step(self, v_meas: float, a_min: float = -5.0, a_max : float = 5.0) -> float:
    # Reference speed at current sample (hold last value after the end)
    target_vel = self.v[self.k] if self.k < self.n else self.v[-1]

    # PID
    e = target_vel - v_meas
    self.int_e += e * self.dt
    de = 0.0 if self._first else (e - self.prev_e) / self.dt
    self._first = False
    self.prev_e = e

    a_cmd = self.kp * e + self.ki * self.int_e + self.kd * de

    # Clip to current accel limits
    a_cmd = float(np.clip(a_cmd, a_min, a_max))

    # Advance time index (20 Hz)
    self.k += 1
    return a_cmd, float(target_vel)
