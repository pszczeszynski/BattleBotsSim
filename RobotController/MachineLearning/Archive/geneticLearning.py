import os
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import numpy as np
import pygad
import tensorflow as tf
tf.compat.v1.disable_eager_execution()

import tensorflow.keras
import pygad.kerasga

from keras.models import Sequential, clone_model
from keras.layers import Dense
import Simulation

input_dim = 1
weights_name = 'weights.h5'


recorded_weights = []


# Define the fitness function
def fitness_func(ga: pygad.GA, solution, sol_idx):
    global recorded_weights
    # reset recorded weights if new generation
    if sol_idx == 1:
        recorded_weights = []

    model.set_weights(pygad.kerasga.model_weights_as_matrix(model, solution))

    # get current generation number
    generation = ga.generations_completed

    if len(recorded_weights) > 0:
        # clone the existing model
        old_model = clone_model(model)
        # choose random index
        index = np.random.randint(0, len(recorded_weights))
        old_model.set_weights(recorded_weights[index])
        score = Simulation.simulate(old_model, model)

        recorded_weights.append(model.get_weights())
        return score
    else:
        recorded_weights.append(model.get_weights())
        return 0

def callback_generation(ga_instance):
    print("Generation = {generation}".format(generation=ga_instance.generations_completed))
    print("Fitness    = {fitness}".format(fitness=ga_instance.best_solution()[1]))
    # plot the fitness
    ga_instance.plot_fitness(title="Fitness over Generations", linewidth=4)

# Create the Keras model
input_layer  = tensorflow.keras.layers.Input(input_dim)
dense_layer1 = tensorflow.keras.layers.Dense(5, activation="relu")(input_layer)
output_layer = tensorflow.keras.layers.Dense(2, activation="linear")(dense_layer1)
model = tensorflow.keras.Model(inputs=input_layer, outputs=output_layer)
keras_ga = pygad.kerasga.KerasGA(model=model,
                                 num_solutions=10)

# Compile the model
model.compile(loss='mse', optimizer='adam')

# load weights
try:
    model.load_weights(weights_name)
except:
    print("No weights found, starting from scratch")

# Set up the PyGAD optimizer
num_parents_mating = 5
num_generations = 200000

# Create the initial population
init_range_low = -1
init_range_high = 1
parent_selection_type = "rank"
crossover_type = "uniform"
mutation_type = "random"
mutation_percent_genes = 2
initial_population = keras_ga.population_weights # Initial population of network weights


ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating,
                       initial_population=initial_population,
                       fitness_func=fitness_func,
                       on_generation=callback_generation)

# Run the genetic algorithm optimization
ga_instance.run()

# Obtain the best solution after optimization
best_solution, best_solution_fitness = ga_instance.best_solution()

# Set the weights of the Keras model with the best solution
model.set_weights(best_solution)

# save the weights  
model.save_weights(weights_name)
