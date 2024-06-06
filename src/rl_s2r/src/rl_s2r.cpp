#include "torch/torch.h"
#include "torch/script.h"
#include <iostream>
#include <chrono>

int main(int argc, char** argv){
  torch::jit::script::Module module;
  module = torch::jit::load(std::string(POLICY_PATH) + "/policy_bl.pt");   
  at::Tensor output;
  auto start = std::chrono::high_resolution_clock::now();
  for(int i = 0; i < 10000; i ++){  
     std::vector<torch::jit::IValue> inputs;
     inputs.push_back(torch::rand({1, 705}));
     output = module.forward(inputs).toTensor();
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << output << std::endl;
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "Program took " << duration << " seconds to run." << std::endl;
}
