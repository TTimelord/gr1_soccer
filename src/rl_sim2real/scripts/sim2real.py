import torch
import torchvision.models as models
import time
device = torch.device("cpu")
model = torch.jit.load('../policy_box/policy_bl.pt',device)
input_tensor = torch.rand(1, 705)
t1 = time.time()
output = model(input_tensor)
t2 = time.time()
print(output)
print(t2 - t1)
