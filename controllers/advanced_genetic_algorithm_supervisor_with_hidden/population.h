#ifndef POPULATION_H
#define POPULATION_H

//   Description:   Genotype population for genetic algorithm

#include "genotype.h"

// abstract type definition
typedef struct _Population_ *Population;

// create a population with specified size and genome size
Population population_create(int pop_size, int gen_size);

// release memory associated with population p
void population_destroy(Population p);

// compute the average fitness of population p
double population_compute_average_fitness(Population p);

// get the fittest genotype of population p
Genotype population_get_fittest(Population p);
  
// get a genotype by its index
Genotype population_get_genotype(Population p, int index);

// change generation: sort the genotypes by fitness and create
// new genotypes using the genotype's crossover and mutation operations
void population_reproduce(Population p);

#endif
