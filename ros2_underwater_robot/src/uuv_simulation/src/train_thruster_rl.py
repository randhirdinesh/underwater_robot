import torch
import torch.optim as optim
import torch.nn as nn
import numpy as np
import random

# Define AI Model
class ThrusterRLModel(nn.Module):
    def __init__(self):
        super(ThrusterRLModel, self).__init__()
        self.fc1 = nn.Linear(4, 16)
        self.fc2 = nn.Linear(16, 16)
        self.fc3 = nn.Linear(16, 3)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return torch.tanh(self.fc3(x))

# Train AI Model with Reinforcement Learning
def train_model():
    model = ThrusterRLModel()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    loss_fn = nn.MSELoss()

    # Simulated Training Data (State: [Depth, Accel_X, Accel_Y, Accel_Z] → Thrust Adjustments)
    training_data = [
        ([5.0, 0.1, -0.2, 0.05], [0.5, -0.3, 0.1]),  # Example training samples
        ([10.0, -0.1, 0.0, -0.05], [-0.2, 0.4, -0.3]),
        ([2.0, 0.2, 0.1, 0.1], [0.7, -0.1, 0.2]),
    ]

    for epoch in range(500):
        random.shuffle(training_data)
        total_loss = 0.0
        for state, target in training_data:
            state_tensor = torch.tensor(state, dtype=torch.float32)
            target_tensor = torch.tensor(target, dtype=torch.float32)

            optimizer.zero_grad()
            output = model(state_tensor)
            loss = loss_fn(output, target_tensor)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        if epoch % 50 == 0:
            print(f"Epoch {epoch}: Loss {total_loss:.4f}")

    torch.save(model.state_dict(), "/home/user/ros2_ws/models/thruster_rl_model.pth")
    print("Model saved!")

if __name__ == "__main__":
    train_model()
