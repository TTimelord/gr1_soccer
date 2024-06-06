#include "rl_s2r.h"
#include <chrono>

GR1S2R::GR1S2R() {
    this->rl_policy_module = torch::jit::load(this->rl_policy_path);
}

GR1S2R::~GR1S2R() {}

void GR1S2R::get_inputs(std::vector<float>& vector_inputs)
{
  at::Tensor tensor_inputs = at::from_blob(vector_inputs.data(), {1,705} ,at::TensorOptions().dtype(at::kFloat));
  this->rl_policy_inputs.push_back(tensor_inputs);  
}

std::vector<float> GR1S2R::get_output()
{
  this->rl_policy_output = this->rl_policy_module.forward(this->rl_policy_inputs).toTensor();
  std::vector<float> vector_output(this->rl_policy_output.data_ptr<float>(), 
                                    this->rl_policy_output.data_ptr<float>() + this->rl_policy_output.numel());
  this->rl_policy_inputs.pop_back();
  return vector_output;

}

void GR1S2R::caculate_and_record()
{

}

int main(int argc, char** argv){
  GR1S2R model;
  std::vector<float> test_data(705, 0.0);
  std::vector<float> outs_data(705);
  auto start = std::chrono::high_resolution_clock::now();
  for(int i = 0; i < 10000; i ++){  
      model.get_inputs(test_data);
      outs_data = model.get_output();
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << outs_data << std::endl;
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "Program took " << duration << " millseconds to run." << std::endl;
}
