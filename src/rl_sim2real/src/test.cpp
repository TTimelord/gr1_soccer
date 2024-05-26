#include "torch/torch.h"
#include "torch/script.h"
#include <iostream>
#include <chrono>

int main(){
  torch::jit::script::Module module;
  module = torch::jit::load("../policy_box/policy_bl.pt"); 
  

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(torch::rand({1, 705}));
  
  auto start = std::chrono::high_resolution_clock::now();  
  at::Tensor output = module.forward(inputs).toTensor();
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << output << std::endl;
  
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  std::cout << "Program took " << duration << " seconds to run." << std::endl;
}
