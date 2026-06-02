import yaml
import ultralytics
import torch

original_torch_load = torch.load

# Create a wrapper that forces map_location to 'cpu'
def forced_cpu_load(*args, **kwargs):
    kwargs['map_location'] = torch.device('cpu')
    return original_torch_load(*args, **kwargs)

# Monkey-patch torch.load
torch.load = forced_cpu_load
# ---------------

# Now PyYAML will safely load the CUDA tensors onto your CPU
with open('res.yaml', 'r') as f:
    data = yaml.load(f, Loader=yaml.UnsafeLoader)

print(data)

