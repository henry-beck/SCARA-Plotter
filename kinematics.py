"""Planar 2-link SCARA inverse/forward kinematics. Angles in radians, lengths in mm."""
import math


class ScaraArm:
    def __init__(self, l1, l2):
        self.l1 = l1
        self.l2 = l2

    def reachable(self, x, y):
        r = math.hypot(x, y)
        return abs(self.l1 - self.l2) <= r <= self.l1 + self.l2

    def ik(self, x, y):
        """Returns [(theta1, theta2), ...] for the elbow-down and elbow-up
        solutions, or [] if (x, y) is outside the arm's reach."""
        if not self.reachable(x, y):
            return []

        l1, l2 = self.l1, self.l2
        r2 = x * x + y * y
        cos_theta2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # clamp float error at the reach boundary

        solutions = []
        for sign in (1.0, -1.0):
            theta2 = sign * math.acos(cos_theta2)
            theta1 = math.atan2(y, x) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
            solutions.append((theta1, theta2))
        return solutions

    def fk(self, theta1, theta2):
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)
        return (x, y)