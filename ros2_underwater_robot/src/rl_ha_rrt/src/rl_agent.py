import torch
import torch.nn as nn
import torch.optim as optim

class RLAgent(nn.Module):
    def __init__(self, state_dim=4, action_dim=8, lr=0.0005):
        super(RLAgent, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 256)
        self.fc3 = nn.Linear(256, action_dim)
        self.optmizer = optim.Adam(self.parameters(), lr=lr)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)
    
    def optimize_path(self, path):
        optimized_path =[]
        for p in path:
            state = torch.FloatTensor([p[0], p[1], 10, 10])
            action = torch.argmax(self.forward(state)).item()
            optimized_path.append((p[0] + action % 3 -1, p[1] +action // 3 -1))
        return optimized_path   # Placeholder : fucture work to integrate RL - based Optimization 