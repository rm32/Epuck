//   Description:   Supervisor code for genetic algorithm

#include "genotype.h"
#include "population.h"
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/display.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/differential_wheels.h>
#include <stdlib.h>

#define NUM_GROUND_SENSORS 3
#define NUM_TOP_SENSORS 8
#define NUM_SENSORS (NUM_GROUND_SENSORS + NUM_TOP_SENSORS)
#define NUM_WHEELS 2
#define HIDDEN 4
#define GENOTYPE_SIZE ((HIDDEN*INPUT) +(HIDDEN*OUTPUT))
#define INPUT (NUM_SENSORS +2)
#define OUTPUT NUM_WHEELS
#define time 600

static const int POPULATION_SIZE = 50;
static const int NUM_GENERATIONS = 20;
static const char *FILE_NAME = "fittest.txt";

static int time_step;
static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag receiver;  //to receive the sensor values from the robot controller

static WbDeviceTag display;   // to display the fitness evolution
static int display_width, display_height;

// the GA population
static Population population;

bool demo = true; 

// for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;
static double robot_trans0[3];  // a translation needs 3 doubles
static double robot_rot0[4];    // a rotation needs 4 doubles

// run the robot simulation for the specified number of seconds
void run_seconds(double seconds) {
    int i, n = 1000.0 * seconds / time_step;
    for (i = 0; i < n; i++) {
        wb_robot_step(time_step);
    }
}



// ------- METHOD USED FOR GRAPH OF FITNESS OVER GENERATIONS -------
void draw_scaled_line(int generation, double y1, double y2) {
  const double XSCALE = (double)display_width / NUM_GENERATIONS;
  const double YSCALE = 0.5; //10.0;
  
  wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE,
   (generation + 0.5) * XSCALE, display_height - y2 * YSCALE);
}

// plot best and average fitness
void plot_fitness(int generation, double best_fitness, double average_fitness) {
  static double prev_best_fitness = 0.0;
  static double prev_average_fitness = 0.0;

  if (generation > 0) {  
  
    wb_display_set_color(display, 0xff0000); // red
    draw_scaled_line(generation, prev_best_fitness, best_fitness);
    
    wb_display_set_color(display, 0x00ff00); // green
    draw_scaled_line(generation, prev_average_fitness, average_fitness);
  }

  prev_best_fitness = best_fitness;
  prev_average_fitness = average_fitness;
}

//--------------------------




// compute fitness
double measure_fitness() {
  double fitness = 0.0;
  double data_received[5];
  
   if (wb_receiver_get_queue_length(receiver) > 0) {

    memcpy(data_received, wb_receiver_get_data(receiver), 5 * sizeof(double));
    
    //Initialize counter variables for reward/punishment
    double fallPunish = data_received[0];
    double sumPunish = data_received[1];
    double speedPunish = data_received[2];
    double reward = data_received[3];
    double circlePunish = data_received[4];
    
    //If the robot goes in circles this will be possitive. We discard if the value is negative
    if (circlePunish < 0)
      circlePunish = 0;

    //Fitness score calculation. Afterwards we scale it so the numbers make more sense.
    double punish = ((double)(0.6 * (double) sumPunish) +(double)(0.3*(double)speedPunish) + (double)(1.5* (double)fallPunish) + (double)(0.5*(double)circlePunish));
    fitness = (((reward *0.5) - punish) + 10000) /100;

    if(fitness < 0)
    {
      fitness = 0; 
    }
     wb_receiver_next_packet(receiver);
  }
 
  return fitness;
}


// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {

  // send genotype to robot for evaluation
  wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));
  robot_trans0[1] = 0;
  
  wb_supervisor_field_set_sf_vec3f(robot_translation, robot_trans0);
  wb_supervisor_field_set_sf_rotation(robot_rotation, robot_rot0);

  // evaluation genotype during a set period of time
  run_seconds(time);
  
  // measure fitness
  double fitness = measure_fitness();
  genotype_set_fitness(genotype, fitness);

  printf("fitness: %g\n", fitness);
}


void run_optimization() {
  wb_robot_keyboard_disable();
  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);

  for  (int i = 0; i < NUM_GENERATIONS; i++) {
    for (int j = 0; j < POPULATION_SIZE; j++) {
      printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype);
    }
  
    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    double average_fitness = population_compute_average_fitness(population);
    
    // display results
    plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    printf("average fitness: %g\n", average_fitness);
    
    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }
  
  printf("GA optimization terminated.\n");

  // save fittest individual
  Genotype fittest = population_get_fittest(population);
  FILE *outfile = fopen(FILE_NAME, "w");
  if (outfile) {
    genotype_fwrite(fittest, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", FILE_NAME);
  }
  else
    printf("unable to write %s\n", FILE_NAME);
  
  population_destroy(population);
}



// show demo of the fittest individual
void run_demo() {
  wb_robot_keyboard_enable(time_step);
  
  printf("---\n");
  printf("running demo of best individual ...\n");
  printf("select the 3D window and push the 'O' key\n");
  printf("to start genetic algorithm optimization\n");

  FILE *infile = fopen(FILE_NAME, "r");
  if (! infile) {
    printf("unable to read %s\n", FILE_NAME);
    return;
  }
  
  Genotype genotype = genotype_create();
  genotype_fread(genotype, infile);
  fclose(infile);
  
  while (demo)
    evaluate_genotype(genotype);
 
}




int main(int argc, const char *argv[]) {
  
  // initialize Webots
  wb_robot_init();
  
  // get simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // the emitter to send genotype to robot
  emitter = wb_robot_get_device("emitter");

  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  // to display the fitness evolution
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_draw_text(display, "fitness", 2, 2);

  // initial population
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
  
  // find robot node and store initial position and orientation
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_rotation = wb_supervisor_node_get_field(robot, "rotation");
  memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
  memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));

  if(demo)
    run_demo();

  // run GA optimization
  run_optimization();
  
  // cleanup Webots
  wb_robot_cleanup();
  return 0;  // ignored
}
