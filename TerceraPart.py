import numpy as np

learning_rate = 0.25
epochs = 5000

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

# Sigmoid a BackPropagation
def sigmoid_derivative(x):
    return x * (1 - x)

#Actualitzar els pesos en BackPropagation
def upd_weight(inputs, weights, delta , learning_rate):
    return weights + np.dot(inputs.T, delta) * learning_rate

#Actualitzar la bias en BackPropagation
def upd_bias(bias,delta, learning_rate):
    return bias + np.sum(delta, axis=0, keepdims=True) * learning_rate

def forward(inputs, weights, bias):
    input_data = np.dot(inputs, weights)+bias
    output = sigmoid(input_data)
    return output

class LayerNeuron:
    def __init__(self, num_inputs, num_neuron):
        self.weights = np.random.rand(num_inputs, num_neuron)
        self.bias = np.ones((1,num_neuron))

    def forward(self, inputs):
        self.input = inputs
        self.output = sigmoid(np.dot(inputs, self.weights) + self.bias)
        return self.output
    def upd_weight_bias(self, delta, learning_rate):
        weight_upd = np.dot(self.input.T,delta) * learning_rate
        self.weights += weight_upd
        self.bias += np.sum(delta, axis=0, keepdims=True) * learning_rate
class XarxaNeuronal:
    def __init__(self, input_size, hidden_size, output_size):
        self.hidden_layer = LayerNeuron(input_size, hidden_size)
        self.output_layer = LayerNeuron(hidden_size, output_size)

    def train(self, inputs, target):
        for i in range(epochs):
            hidden_output = self.hidden_layer.forward(inputs)
            output = self.output_layer.forward(hidden_output)

            # calcul delta layer 2
            error_output = target - output
            delta_output = error_output * sigmoid_derivative(output)

             # calcul delta layer 1
            layer_error = delta_output.dot(self.output_layer.weights.T)
            layer_delta = layer_error * sigmoid_derivative(hidden_output)

             # Actualització pesos layer 2
            self.output_layer.upd_weight_bias(delta_output, learning_rate)

            # Actualització pesos layer 1
            self.hidden_layer.upd_weight_bias(layer_delta, learning_rate)
        print("Salida deseada:", target)
        print("Salida Puertas(XOR, AND):")
        print(output)


inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
targets = np.array([[0, 0], [1, 0], [1, 0], [0, 1]])

# Initialize the neural network with 2 inputs, 3 hidden neurons, and 2 output neurons
xarxa_neuronal = XarxaNeuronal(2, 3, 2)
xarxa_neuronal.train(inputs, targets)
