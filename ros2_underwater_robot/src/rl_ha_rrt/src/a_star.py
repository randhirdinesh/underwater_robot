import heapq

class AStarPlanner:
    def __init__(self):
        self.directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]

        
    def heusistic(self, node, goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start : 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:

            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for dx, dy in [(0,1), (1,0), (0.-1), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] +dy)
                tentative_g_score = g_score[current] + 1.41 if dx != 0 and dy !=0 else 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heurstic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return[]
    def reconstruct_path(self, came_from , current ):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


            