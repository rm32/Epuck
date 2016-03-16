// Description:   Robot execution code for genetic algorithm

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define NUM_SENSORS 8
#define NUM_WHEELS 2
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS) + 4

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS + 2][NUM_WHEELS]; 
double previous_wheels_speed[NUM_WHEELS];

WbDeviceTag sensors[NUM_SENSORS];  // proximity sensors
WbDeviceTag receiver;              // for receiving genes from Supervisor
WbDeviceTag emitter;                // for sending the fitness value to the supervisor

//Based on http://wwwold.ece.utep.edu/research/webfuzzy/docs/kk-thesis/kk-thesis-html/node72.html
double sigmoid(double x)
{
     double exp_value;
     double return_value;

     /*** Exponential calculation ***/
     exp_value = exp((double) -x);

     /*** Final sigmoid value ***/
     return_value = 1 / (1 + exp_value);

     return return_value;
}


// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));

    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver);
  }
}

static double clip_value(double value, double min_max) {
  if (value > min_max)
    return min_max;
  else if (value < -min_max)
    return -min_max;

  return value;
}

void sense_compute_and_actuate() {
  // read sensor values
  double sensor_values[NUM_SENSORS];
  double* data_emitted = malloc(10 * sizeof(double));
  int i, j;
  for (i = 0; i < NUM_SENSORS; i++){

    //input neurons:
    sensor_values[i] += wb_distance_sensor_get_value(sensors[i]);
    //printf("sensor %d %f\n", i, sensor_values[i]);  
  }
  
  memcpy(data_emitted, sensor_values, 8 * sizeof(double));
  
  //TODO: HERE GOES THE RNN INSTEAD!!!!
  //output neurons:
  double wheel_speed[NUM_WHEELS] = { 0.0, 0.0 };
  double sum = 0.0;
  for (i = 0; i < NUM_WHEELS; i++){
    for (j = 0; j < NUM_SENSORS; j++){
    	sum += matrix[j][i] * sensor_values[j];
    }

    //add the recurrent connections on the output layer from the previous time step
    sum += matrix[NUM_SENSORS][i] * previous_wheels_speed[i];
    sum += matrix[NUM_SENSORS + 1][i] * previous_wheels_speed[(i + 1)%2];

    //apply the activation function to the weighted inputs
    wheel_speed[i] = 400 * sigmoid(sum);
  }  

  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);

  //Append wheel speed to data_emitter
  memcpy(data_emitted + 8, wheel_speed, 2 * sizeof(float));
  printf("left wheel: %f\n", wheel_speed[0]);
  printf("right wheel: %f\n", wheel_speed[1]);

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(wheel_speed[0], wheel_speed[1]);
  previous_wheels_speed[0] = wheel_speed[0];
  previous_wheels_speed[1] = wheel_speed[1];
  
  // send sensor values to supervisor for evaluation
  wb_emitter_send(emitter, data_emitted, (NUM_SENSORS + NUM_WHEELS) * sizeof(double));
}

int main(int argc, const char *argv[]) {
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();
    
  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NUM_SENSORS; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }
    
  // the emitter to send fitness value to supervisor
  emitter = wb_robot_get_device("emitter");

  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  // initialize matrix to zero, hence the robot 
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));
  
  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
    check_for_new_genes();
    sense_compute_and_actuate();
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
