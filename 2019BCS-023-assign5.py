import math
import numpy as np


class FABRIKSolver:
    def __init__(self, joints: list, tolerance: float):
        self.joints = joints
        self.tolerance = tolerance
        self.link_lengths = []

        joint1 = joints[0]
        for j in joints[1:]:
            self.link_lengths.append(np.linalg.norm(joint1 - j))
            joint1 = j

        self.lengths = self.link_lengths
        self.max_len = sum(self.link_lengths)

        self.moved = True
        self._angles = []
        _ = self.angles

    def angles(self) -> list:
        if not self.moved:
            return self._angles

        angles = [math.atan2(self.joints[1][1], self.joints[1][0])]

        prev_angle = angles[0]
        for i in range(2, len(self.joints)):
            p = self.joints[i] - self.joints[i - 1]
            abs_angle = math.atan2(p[1], p[0])
            angles.append(abs_angle - prev_angle)
            prev_angle = abs_angle

        self.moved = False
        self._angles = angles
        return self._angles

    def getLength(self, vector, length) -> float:
        return vector * length / np.linalg.norm(vector)

    def isSolvable(self, target):
        return self.max_len >= np.linalg.norm(target)

    def anglesInDegrees(self):
        return [math.degrees(val) for val in self.angles()]

    def move(self, target, try_to_reach=True):
        if not self.isSolvable(target):
            if not try_to_reach:
                return 0
            target = self.getLength(target, self.max_len)
        return self.iterate(target)

    def iterate(self, target):
        it_count = 0
        initial_position = self.joints[0]
        last = len(self.joints) - 1

        while np.linalg.norm(self.joints[-1] - target) > self.tolerance:
            it_count += 1

            self.joints[-1] = target
            for i in reversed(range(0, last)):
                next_, current = self.joints[i + 1], self.joints[i]
                len_share = self.lengths[i] / np.linalg.norm(next_ - current)
                self.joints[i] = (1 - len_share) * next_ + len_share * current

            self.joints[0] = initial_position
            for i in range(0, last):
                next_, current = self.joints[i + 1], self.joints[i]
                len_share = self.lengths[i] / np.linalg.norm(next_ - current)
                self.joints[i + 1] = (1 - len_share) * \
                    current + len_share * next_
        return it_count


print("Enter joint positions")
j1 = np.array(
    list(map(int, input("Joint 1: ").strip().split(),))[:3])

j2 = np.array(list(
    map(int, input("Joint 2: ").strip().split(),))[:3])

j3 = np.array(
    list(map(int, input("Joint 3: ").strip().split()))[:3])

j4 = np.array(list(
    map(int, input("Joint 4: ").strip().split(),))[:3])

tolerance = float(input("Enter Tolerance: "))
goal = np.array(
    list(map(int, input("Goal point: ").strip().split(),))[:3])

intial_coord = [j1, j2, j3, j4]
robot = FABRIKSolver(intial_coord, tolerance)

iterations = robot.move(np.array(goal))

print("Iterations: ", iterations)
print("Angles:", robot.anglesInDegrees())
print("Link position:", robot.joints)
print("Goal Position:", goal)
