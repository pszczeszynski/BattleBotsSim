import numpy as np
import pygad
import keras
from keras.models import Sequential
from keras.layers import Dense


input_dim = 8
weights_name = 'weights.h5'

# Define the fitness function
def fitness_func(solution, sol_idx):
    model.set_weights(solution)

    # run simulation and then return fitness (from 0 to 1)

    return fitness

# Create the Keras model
model = Sequential()
model.add(Dense(64, activation='relu', input_shape=(input_dim,)))
model.add(Dense(32, activation='relu'))
model.add(Dense(1, activation='linear'))

# Compile the model
model.compile(loss='mse', optimizer='adam')

# load weights
try:
    model.load_weights(weights_name)
except:
    print("No weights found, starting from scratch")

# Set up the PyGAD optimizer
num_solutions = 10
num_parents_mating = 5
num_generations = 20

# Create the initial population
init_range_low = -2
init_range_high = 5
parent_selection_type = "rank"
crossover_type = "single_point"
mutation_type = "random"
mutation_percent_genes = 10

# Initialize the population
population_size = (num_solutions, model.count_params())
population = pygad.initial_population(range_low=init_range_low, range_high=init_range_high, size=population_size)

# Create an instance of the pygad.GA class
ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating,
                       fitness_func=fitness_func,
                       sol_per_pop=num_solutions,
                       num_genes=model.count_params(),
                       init_range_low=init_range_low,
                       init_range_high=init_range_high,
                       parent_selection_type=parent_selection_type,
                       crossover_type=crossover_type,
                       mutation_type=mutation_type,
                       mutation_percent_genes=mutation_percent_genes)

# Run the genetic algorithm optimization
ga_instance.run(population)

# Obtain the best solution after optimization
best_solution, best_solution_fitness = ga_instance.best_solution()

# Set the weights of the Keras model with the best solution
model.set_weights(best_solution)

# save the weights  
model.save_weights(weights_name)
