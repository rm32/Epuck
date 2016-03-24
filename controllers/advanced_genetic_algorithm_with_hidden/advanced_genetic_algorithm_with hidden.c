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

#define NUM_SENSORS 10
#define NUM_WHEELS 2
#define GENOTYPE_SIZE ((HIDDEN * (NUM_SENSORS + 2)) + (NUM_WHEELS * HIDDEN))
#define RANGE (1024/2)
#define HIDDEN 10

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS + 2][NUM_WHEELS]; 
double ctx[NUM_WHEELS];
double ctx[NUM_WHEELS];
int time_step;
int emitter_counter, steps;
double sensor_values[NUM_SENSORS];
double mean_sensor_values[NUM_SENSORS - 2];
// read sensor values
double* data_emitted;
//output neurons:
double wheel_speed[NUM_WHEELS] = { 0.0, 0.0 };
double mean_wheel_speed[NUM_WHEELS];

double matrix2[GENOTYPE_SIZE];
double hiddenAct[HIDDEN];

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
printf("here 1");
  if (wb_receiver_get_queue_length(receiver) > 0) {
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(matrix2, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));
    for (i=0;i<GENOTYPE_SIZE;i++){
      printf("matrix %f, ", matrix2[i]);
    }

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
  
  int i, j, k;
  for (i = 0; i < NUM_SENSORS - 2; i++){
    //get the sumed value of each sensor so we can calculate the mean value before sending them back
    mean_sensor_values[i] += wb_distance_sensor_get_value(sensors[i]);
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
    //printf("sensor %d %f\n", i, sensor_values[i]);  
  }
  for (i = 8; i < NUM_SENSORS; i++){
     
    if (wb_distance_sensor_get_value(sensors[i]) > 800){
    //printf("sensor %d %f\n", i, wb_distance_sensor_get_value(sensors[i])); 
      sensor_values[i] += -1.0;
    }else{
      sensor_values[i] += 0.0;
    }
  }
  
  int ofs = 0;
  double sum = 0.0;
  // for (i = 0; i < NUM_WHEELS; i++){
  //   wheel_speed[i] =  0.0;
  //   for (j = 0; j < NUM_SENSORS; j++){
  //   	sum += matrix[j][i] * (1.0 - (sensor_values[j] / RANGE));
  //   }

  //   //add the recurrent connections on the output layer from the previous time step
  //   sum += matrix[NUM_SENSORS][i] * (1.0 - (ctx[i] / RANGE));
  //   sum += matrix[NUM_SENSORS + 1][i] * (1.0 - (ctx[(i + 1)%2] / RANGE));
  //   //sum += matrix[NUM_SENSORS][i] * ctx[i];
  //   //sum += matrix[NUM_SENSORS + 1][i] * ctx[(i + 1)%2];

  //   //apply the activation function to the weighted inputs
  //   wheel_speed[i] = 500 + 1000*tanh(sum);
  // }  

  for (i = 0; i < NUM_WHEELS; i++){
    wheel_speed[i] =  0.0;
    for (j = 0; j < HIDDEN; j++){
      for (k = 0; k < NUM_SENSORS; k++){
        sum += matrix2[ofs] * (1.0 - (sensor_values[j] / RANGE)); //TODO: recenter the sensor value 
        ofs++;
      }
      sum += matrix2[ofs] * (1.0 - (ctx[i] / RANGE));
      ofs++;
      sum += matrix2[ofs] * (1.0 - (ctx[(i + 1)%2] / RANGE));
      ofs++;

      hiddenAct[j] = tanh(sum);
      sum = 0.0;
    }
    for (j = 0; j < HIDDEN; j++){
      sum += matrix2[ofs] * hiddenAct[j];
      ofs++;
    }

    wheel_speed[i] = tanh(sum);
    printf("wheel %d %f\n",i,wheel_speed[i] );
    ctx[i] = wheel_speed[i];
  }


  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);
  
  //printf("w0: %f and w1: %f\n", wheel_speed[0], wheel_speed[1]);

  //accumulate wheel speeds to calculate the average afterwards
  mean_wheel_speed[0] += wheel_speed[0];
  mean_wheel_speed[1] += wheel_speed[1];

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(500*wheel_speed[0], 500*wheel_speed[1]);

  //save current speeds to be used as previous speed on next run
  ctx[0] = wheel_speed[0];
  ctx[1] = wheel_speed[1];
  
}

int main(int argc, const char *argv[]) {
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  time_step = wb_robot_get_basic_time_step();
  printf("time step: %d\n",time_step);
    
  //initialize the emitter counter to send data back to the supervisor  
  emitter_counter = 60000/time_step;
printf("size: %d\n",GENOTYPE_SIZE);
  //copy emitter counter to a variable to use the initial value later
  steps = emitter_counter;

  // find and enable proximity sensors
  char name[32];
  int i;
 
   for (i = 0; i < 8; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }
  
   for (i = 0; i < 2; i++) {
    sprintf(name, "gs%d", i);
    sensors[i + 8] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i + 8], time_step);
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
  printf("main_");
    check_for_new_genes();
    sense_compute_and_actuate();
    
    emitter_counter--;
    printf("step: %d\n", emitter_counter);
    
    if (emitter_counter == 0){
      data_emitted = malloc((NUM_SENSORS + NUM_WHEELS) * sizeof(double));

      //calculate the mean value of each wheel
      mean_wheel_speed[0] = mean_wheel_speed[0]/steps;
      mean_wheel_speed[1] = mean_wheel_speed[1]/steps;

      //calculate the mean value of each sensor
      for (i=0; i< NUM_SENSORS-2; i++){
        sensor_values[i] = sensor_values[i]/steps;
        sensor_values[i]  /= 4096; //normalize the sensor values
        //printf("sensor %d value: %f\n",i,sensor_values[i]);
      }
      //printf("sensor %d value: %f\n",8,sensor_values[8]);
      //printf("sensor %d value: %f\n",9,sensor_values[9]);
      memcpy(data_emitted, sensor_values, NUM_SENSORS * sizeof(double));

      //Append wheel speed to data_emitter
      memcpy(data_emitted + NUM_SENSORS, mean_wheel_speed, NUM_WHEELS * sizeof(double));
      //printf("data emitted: %f\n",data_emitted);
      
      // send data to supervisor for evaluation and reset the counter
      wb_emitter_send(emitter, data_emitted, (NUM_SENSORS + NUM_WHEELS) * sizeof(double));
      emitter_counter = steps;
      
      for (i = 0; i < NUM_SENSORS; i++){
        sensor_values[i] = 0.0;
      }
    }
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
