import numpy as np
def sigmoid(x):
    return 1 / (1 + np.exp(-x))
class LayerNeuron:
    def __init__(self, num_inputs, num_neuron):
        self.weights = np.ones((num_inputs, num_neuron))
        self.bias = np.ones(num_neuron)
    def forward(self, inputs):
        sumaEntrades = np.dot(inputs, self.weights) + self.bias
        return sigmoid(sumaEntrades)
class xarxaNeuronal:
    def __init__(self, num_inputs, num_neurones_amagades, num_output_neuron):
        self.hidden_layer = LayerNeuron(num_inputs, num_neurones_amagades)
        self.output_layer = LayerNeuron(num_neurones_amagades, num_output_neuron)
    def forward(self, inputs):
        hidden_output = self.hidden_layer.forward(inputs)
        output = self.output_layer.forward(hidden_output)
        return output
red = xarxaNeuronal(2, 3, 2)
input_data = np.array([0, 1])
output = red.forward(input_data)
print("Sortida:", output)
