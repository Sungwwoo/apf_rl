import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical
import numpy as np
from torch.utils.tensorboard import SummaryWriter

device = torch.device("cpu")
if torch.cuda.is_available():
    device = torch.device("cuda:0")
    torch.cuda.empty_cache()


class RolloutBuffer:
    def __init__(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.log_progs = []
        self.dones = []
        self.values = []

    def save(self):
        return

    def flush(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.log_progs = []
        self.dones = []
        self.values = []


# Policy network
class ActorCritic(nn.Module):
    def __init__(self, input_dim, action_dim):
        super(ActorCritic, self).__init__()

        # Shared layer
        self.sharedLayer = nn.Linear(input_dim, 512)

        # Action network: output dim = action dim
        self.actor = nn.Sequential(
            self.sharedLayer,
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, action_dim),
        ).to(device)

        # Value network: output dim = 1
        self.critic = nn.Sequential(
            self.sharedLayer,
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, 1),
        ).to(device)

        # Using multi-discrete action space
        # -> Multiple Categorical distribution required
        self.dist = [Categorical for _ in range(action_dim)]

    def act(self, state):
        action_probs = self.actor(state)

    def evaluate_action(self, state, action): ...


class PPO:
    def __init__(self, state_dim, action_dim, lr, epsilon, gamma, epochs):
        # gamma, clip range, policy, optimizer,
        self.buffer = RolloutBuffer()
        self.policy = ActorCritic()
        return

    def get_action(self): ...

    def update(self, state):
        # Convert state to tensor

        # get action

        return

    def save(self, file): ...

    def load(self, file): ...
