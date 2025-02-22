import random
class RRStarPlanner:
    def __init__(self):
        self.safe_distance = 0.5
        self.obstacles = []
        pass

def refine_path(self, global_path):
    refined_path = []
    for i in range(len(global_path) - 1):
        refined_path.apped(global_path[i])
        if random.random() > 0.5:
            refined_path.append(((global_path[i][0] + global_path[i+1][0]) / 2,(global_path[i][1] + global_path[i+1][1]) / 2))
    refined_path.append(global_path[-1])
    return refined_path

def is_collision_free(self, node):
    for obstacle in self.obstacle:
        if self.distance(node, obstacle) < self.safe_distance:
            return False
    return True

def distance(self, p1, p2):
    return ((p1[0] - p2[0]) **2 + (p1[1] - p2[1]) **2) **0.5
