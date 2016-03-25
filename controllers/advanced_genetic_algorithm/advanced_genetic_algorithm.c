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

#define NUM_SENSORS 11
#define NUM_WHEELS 2
#define NUM_GROUND_SENSORS 3
#define GENOTYPE_SIZE ((NUM_SENSORS * NUM_WHEELS) + 4)

// see braitenberg
#define RANGE (1024/2)

// due to topology
double matrix[NUM_SENSORS + 2][NUM_WHEELS];

//ctx - context : this is the values of the previous wheel interations
double ctx[NUM_WHEELS];

int time_step;
int emitter_counter, steps;
double sensor_values[NUM_SENSORS];

double mean_sensor_values[NUM_SENSORS - NUM_GROUND_SENSORS];

//wheel encoder variables
// left and right : current left and right encoder
// ctx_left and ctx_right : previous left and right encoder
double left, right, ctx_left, ctx_right, encoder_change[1];

// read sensor values
double* data_emitted;

//output neurons:
double wheel_speed[NUM_WHEELS] = { 0.0, 0.0 };

//+1 is needed for the encoder
double mean_wheel_speed[NUM_WHEELS +1];

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

//sort values
void sort_values(double values[], int size){
  double temp;
  int i,j;
  
  for (i = 0; i < size; i++){
    for (j = i + 1; j < size; j++){
      if (values[i] < values[j]){
        temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }
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

left = wb_differential_wheels_get_left_encoder();
right = wb_differential_wheels_get_right_encoder();

//COULD JUST GET FINAL VALUES
mean_wheel_speed[2] += (double)(left - ctx_left) + (double)(right - ctx_right); //calculate the change

ctx_left = left;
ctx_right = right;

for (int i = 0; i < NUM_SENSORS - NUM_GROUND_SENSORS; i++){
    
    //get the sumed value of each sensor so we can calculate the mean value before sending them back
    mean_sensor_values[i] += wb_distance_sensor_get_value(sensors[i]);
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
}
    
for (int i = (NUM_SENSORS - NUM_GROUND_SENSORS); i < NUM_SENSORS; i++){
    if (wb_distance_sensor_get_value(sensors[i]) > 800){
      sensor_values[i] += -1.0;
    }
}
  
  
double sum = 0.0;

for (int i = 0; i < NUM_WHEELS; i++){
    
    //setting all the wheel speeds to zero
    wheel_speed[i] =  0.0;
    
    // getting the sum values of all the sensors
    for (int j = 0; j < NUM_SENSORS; j++){
        
        //TODO: need to explain this maths
    	sum += matrix[j][i] * (1.0 - (sensor_values[j] / RANGE));
    }

    //add the recurrent connections on the output layer from the previous time step
    sum += matrix[NUM_SENSORS][i] * (1.0 - (ctx[i] / RANGE));
    sum += matrix[NUM_SENSORS + 1][i] * (1.0 - (ctx[(i + 1)%2] / RANGE));

    //apply the activation function to the weighted inputs
    wheel_speed[i] = tanh(sum);
    sum = 0.0;
  }
    

  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);


  //accumulate wheel speeds to calculate the average afterwards
  mean_wheel_speed[0] += wheel_speed[0];
  mean_wheel_speed[1] += wheel_speed[1];

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(200*wheel_speed[0], 200*wheel_speed[1]);

  //save current speeds to be used as previous speed on next run
  ctx[0] = wheel_speed[0];
  ctx[1] = wheel_speed[1];
  
}





int main(int argc, const char *argv[]) {
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  time_step = wb_robot_get_basic_time_step();
    
  //initialize the emitter counter to send data back to the supervisor  
  emitter_counter = 120000/time_step;

  //copy emitter counter to a variable to use the initial value later
  steps = emitter_counter;


  // find and enable proximity sensors
  char name[32];
    
    // 8 is the number of top sensors
   for (int i = 0; i < (NUM_SENSORS - NUM_GROUND_SENSORS); i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }
  
    // 3 is the current number of used ground sensors
   for (int i = 0; i < NUM_GROUND_SENSORS; i++) {
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
  
  //enable the encoders to measure distance traveled
  wb_differential_wheels_enable_encoders(time_step);
  
  //load_best();
  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
  
    check_for_new_genes();
      
    sense_compute_and_actuate();
    
    emitter_counter--;
    
    if (emitter_counter == 0){

      //reset the wheel encoders and sum
      wb_differential_wheels_set_encoders(0.0, 0.0);
      encoder_change[0] = 0.0;

      data_emitted = malloc((NUM_SENSORS + NUM_WHEELS + 1) * sizeof(double));

      //calculate the mean value of each wheel
      mean_wheel_speed[0] = mean_wheel_speed[0]/steps;
      mean_wheel_speed[1] = mean_wheel_speed[1]/steps;

      //calculate the mean value of each sensor
      for (int i=0; i< NUM_SENSORS-2; i++){
        mean_sensor_values[i] = sensor_values[i]/steps;
        mean_sensor_values[i]  /= 4096; //normalize the sensor values
      }

      memcpy(data_emitted, sensor_values, NUM_SENSORS * sizeof(double));
      
      //Append wheel speed (and encoder) to data_emitter
      memcpy(data_emitted + NUM_SENSORS, mean_wheel_speed, (NUM_WHEELS + 1) * sizeof(double));
      
      // send data to supervisor for evaluation and reset the counter
      wb_emitter_send(emitter, data_emitted, (NUM_SENSORS + NUM_WHEELS + 1) * sizeof(double));
      emitter_counter = steps;
      
      for (int i = 0; i < NUM_SENSORS; i++){
        sensor_values[i] = 0.0;
      }
    }
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
