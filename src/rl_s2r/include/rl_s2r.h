#include "torch/torch.h"
#include "torch/script.h"
#include <iostream>

class GR1S2R
    {
    private:
        std::string rl_policy_path = std::string(POLICY_PATH) + "/policy_bl.pt";
        torch::jit::script::Module rl_policy_module;
        at::Tensor rl_policy_output;
        std::vector<torch::jit::IValue> rl_policy_inputs;

    public:

    private:
        void caculate_and_record();

    public:
        GR1S2R();
        ~GR1S2R();
        void get_inputs(std::vector<float>& vector_inputs);
        std::vector<float> get_output();
};
