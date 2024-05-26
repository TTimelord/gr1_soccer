import torch
import torchvision.models as models
import time
device = torch.device("cpu")
model = torch.jit.load('../policy_box/policy_bl.pt',device)
t1 = time.monotonic()
t2 = time.monotonic()
for i in range(10000):
    input_tensor = torch.rand(1, 705)
    output = model(input_tensor)
t3 = time.monotonic()
print(t3 - t2)
print((t3 - t2)-(t2 - t1))
