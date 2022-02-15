# This is a class contains information and methods with mass.
# parameters:
#               k: a constants double, spring constant
#               l_0: a constants double, spring rest length
#               m_1: first mass connected, a class mass instance
#               m_2: second mass connected, a class mass instance


class spring:
    def __init__(self, l_0: float, m1: int, m2: int, p: list):
        self.l_0 = l_0
        self.m1 = m1
        self.m2 = m2
        self.parameter = p
