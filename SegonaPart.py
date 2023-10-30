import numpy as np

learning_rate = 0.25
epochs = 5000

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def sigmoid_derivative(x):
    return x * (1 - x)

def upd_weight(inputs, weights, delta , learning_rate):
    return weights + np.outer(inputs, delta) * learning_rate

def upd_bias(bias,delta, learning_rate):
    return bias + np.sum(delta, axis=0, keepdims=True) * learning_rate

def forward(inputs, weights, bias):
    input_data = np.dot(inputs, weights)+bias
    output = sigmoid(input_data)
    return output

class XarxaNeuronal:
    def __init__(self, inputs, target):

        pesos_input = np.random.rand(2, 3)
        bias = np.array([1, 1, 1])
        pesos_output = np.random.rand(3, 2)
        bias_output = np.array([1, 1])

        for i in range(epochs):
            hidden_input = forward(inputs, pesos_input, bias)
            output = forward(hidden_input,pesos_output,bias_output)

            error_output = target - output
            delta_output = error_output * sigmoid_derivative(output)

            layer_error = delta_output.dot(pesos_output.T)
            layer_delta = layer_error * sigmoid_derivative(hidden_input)

            pesos_output = upd_weight(hidden_input, pesos_output, delta_output, learning_rate)
            pesos_input = upd_weight(inputs,pesos_input,layer_delta,learning_rate)
            bias_output = upd_bias(bias_output,delta_output,learning_rate)
            bias = upd_bias(bias,layer_delta,learning_rate)

        print("Salida deseada:", target)
        print("Salida Back Propagation:", output)



xarxaNeuronal = XarxaNeuronal(inputs=np.array([0,1]), target= np.array([0,1]))

