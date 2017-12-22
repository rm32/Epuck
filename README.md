# Simulating the behaviour of a Rat in an EPM using an E-Puck

## Design

### Topology
We are using a Recurrent Neural Network (RNN) using the Jordan architecture. The topology consists of 11 input nodes (8 distance sensors plus 3 ground sensors) one hidden layer of 4 neurons, 2 output nodes, each controlling the speed of a wheel and 2 context units. The output nodes at each iteration are copied to these context units. These are then fed back into the hidden layer on the next iteration. As the network's purpose is to plan actions for the actuators of the E-puck, actions already taken (output) must be remembered to provide acceptable results. As previously explained, the context units act as a short-term memory for the network. All the network is fully connected, providing 52 (11 inputs nodes * 4 hidden nodes + 4 hidden nodes * 2 output nodes) weighted connections between the inputs, hidden layer and outputs, and 8 recurrent weighted connections from the context to the hidden layer.

In the above topology, the two output nodes provide the wheel speed at time t, while the context nodes hold the previous output of the network (speed of wheels at time t-1).

The activation function used to fire all nodes is the hyperbolic tangent (tanh), as we want the output to receive negative values as well in order for the wheels to be able to turn backwards. The weights of the network are going to be evolved by the genetic algorithm (GA). The activation function for the neurons in the hidden layer is given by the function,
while for the output neurons, where wij are the weights between the input and the hidden layer, cik the recurrent connections and yt-1k the output of the network at the previous time step.


### Genetic Algorithm
#### Genotype
The genotype is encoded as an array of 60 double values, each representing one of the network's weights (including the recurrent connections). The genotype can take values between –1 and 1.

#### Fitness Function
The fitness function is split up between reward and punishment. The robot is rewarded the further distance it travels and it is punished if it falls off the maze, gets and stays near the walls or continues rotating with small circle radius. The fitness score is represented by the following function:
Where D denotes the distance travelled reward, S the penalty of the summed values of the distance sensors, C represents the penalty for going in small circles and G is the penalty for going outside the maze. a, b, c and d are weights for each individual reward/penalty (e.g. falling off the edge of maze is penalized much heavier than standing near a wall). The way the fitness function is calculated is explained in more detail in the "Implementation" section.
