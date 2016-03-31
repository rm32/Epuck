// Description:   Robot execution code for genetic algorithm

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/accelerometer.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#define NUM_GROUND_SENSORS 3 //3 ground sensors
#define NUM_TOP_SENSORS 8 //8 distance sensors
#define NUM_SENSORS (NUM_GROUND_SENSORS + NUM_TOP_SENSORS)
#define NUM_WHEELS 2
#define HIDDEN 4 //4 nodes in the hidden layer
#define GENOTYPE_SIZE ((HIDDEN*INPUT) +(HIDDEN*OUTPUT)) //60 weights
#define INPUT (NUM_SENSORS +2)
#define OUTPUT NUM_WHEELS

#define RANGE (1024/2)

//Running time
#define time 120

int time_step;
int steps;
int counter;
double sensor_values[NUM_SENSORS];
double sensors[NUM_SENSORS];
double lightSensors[8];
double speed[2] ; 
double ctx[2]; //context variable to hold the recurrent weights

double wheel_speed[2];
double left; 
double right; 
double mean_wheel_speed[3];
double distance[1] = {0.0};

// read sensor values
double* data_emitted;
double matrix[INPUT][HIDDEN];
double matrix2[HIDDEN][OUTPUT];
double hidden[4] = {0,0,0,0};

WbDeviceTag receiver;              // for receiving genes from Supervisor
WbDeviceTag emitter;                // for sending the fitness value to the supervisor

double fitness[5] = {0,0,0,0,0};    //Initialize the reward/punishment counters


// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    double genes[GENOTYPE_SIZE];
   
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(genes, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));

    //Copy the weights between the input and hidden layer
    memcpy(matrix, genes, INPUT*HIDDEN * sizeof(double));
   
    //Copy the weights between the hidden layer and the output layer
    memcpy(matrix2, genes + (INPUT*HIDDEN), OUTPUT*HIDDEN * sizeof(double));
 
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


double old_left = 0; 
double old_right = 0; 
double ctx_leftSpeed = 0;
double ctx_rightSpeed = 0;

void sense_compute_and_actuate() {

  //Get the differential wheel encoder's readings
  left= wb_differential_wheels_get_left_encoder();
  right = wb_differential_wheels_get_right_encoder(); 
  
  //get each wheel speed
  double leftSpeed =  wb_differential_wheels_get_left_speed(); 
  double rightSpeed = wb_differential_wheels_get_right_speed();
  
  
double sum = 0; 
for (int i = 0; i < NUM_SENSORS - NUM_GROUND_SENSORS; i++){
    
    //Get the distance sensor readings and add them up to calculate punishment "S"
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
    sum += sensor_values[i]; 
}

// check to see if the robot has gone outside the black area. 
//For every time step it has, add one to the counter
for (int i = (NUM_SENSORS - NUM_GROUND_SENSORS); i < NUM_SENSORS; i++){
    if (wb_distance_sensor_get_value(sensors[i]) > 800){
      fitness[0]++;
    }
}
  
  //Increase the punishment every time it gets next to a wall.
  if(sum > 5000)
  {
    fitness[1]++;
  }
  
  //Increase the counter for going backwards (complementary with fitness[1] for obstacle avoidance.)
  //The weight in the fitness function is allowing the robot to go backwards if needed for a little while.
  if((leftSpeed < 0) || (rightSpeed < 0) )
  {
    fitness[2]++;
  }
  
  //Increase the reward counter every time the robot has moved from the previous time step
  if(( left > old_left+3) || ( right > old_right+3) )
  {
    fitness[3]++;
  }
  
  //Check if it is going in circles and increase the counter. 
  if( (fabs(leftSpeed - rightSpeed) > 40) && (fabs(ctx_leftSpeed - ctx_rightSpeed) > 40) ){
    fitness[4]++;
  }else{
    fitness[4]--;
    
  }

  //Set the current wheel speeds and wheel encoders as the context for the next iteration.
  old_left = left; 
  old_right = right;  

  ctx_leftSpeed = leftSpeed;
  ctx_rightSpeed = rightSpeed;
  

 for (int i = 0; i < HIDDEN; i++){

    //setting all the wheel speeds to zero
    hidden[i] =  0.0;
    double sum = 0.0;
    
    /*
    * Getting the weighted sum values of input layer. 
    * We need to recenter the value of the sensor to be able to get
    * negative values too. This will allow the wheels to go 
    * backward too. */
    for (int j = 0; j < NUM_SENSORS; j++){
        double weight = matrix[j][i];
        double input = sensor_values[j];
        sum += weight * (1.0 - (input / RANGE));

    }

    //add the recurrent connections on the output layer from the previous time step
    sum += matrix[NUM_SENSORS][i] * (1.0 - (ctx[i%2] / RANGE));
    sum += matrix[NUM_SENSORS + 1][i] * (1.0 - (ctx[(i + 1)%2] / RANGE));

    //apply the activation function to the weighted inputs
    hidden[i] = tanh(sum);
  }
  
  
  for(int i = 0; i < NUM_WHEELS; i++){
    //setting all the wheel speeds to zero
    wheel_speed[i] =  0.0;
    double sum = 0.0;
    
    // Getting the weighted sum values of hidden layer.
    for (int j = 0; j < HIDDEN; j++){
        double weight = matrix2[j][i];
        double input = hidden[j];
      sum += weight * input;
    }
 
    //apply the activation function to the weighted inputs
    wheel_speed[i] = tanh(sum);
  }
  

  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(200*wheel_speed[0], 200* wheel_speed[1]);

  //save current speeds to be used as previous speed on next run
  ctx[0] = wheel_speed[0];
  ctx[1] = wheel_speed[1];
}


int main(int argc, const char *argv[]) {
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  time_step = wb_robot_get_basic_time_step();
    
  //initialize the emitter counter to send data back to the supervisor  
  int emitter_counter = ((time * 1000)/time_step) - 1;

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


  for (int i = 0; i < 8; i++) {
    sprintf(name, "ls%d", i);
    lightSensors[i] = wb_robot_get_device(name);
    wb_light_sensor_enable(lightSensors[i], time_step);
 } 
   
    
  // the emitter to send fitness value to supervisor
  int emitter = wb_robot_get_device("emitter");
  

  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  // initialize matrix to zero, hence the robot 
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));
  
  //enable the encoders to measure distance traveled
  wb_differential_wheels_enable_encoders(time_step);

  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
  
   check_for_new_genes();
      
   sense_compute_and_actuate();
    
   //Decrease the sync counter every 32ms to synchronize the transmitting with the supervisor
   emitter_counter--;
   

    
   if (emitter_counter == 0){
     left= wb_differential_wheels_get_left_encoder(); //get the left wheel encoder
     right = wb_differential_wheels_get_right_encoder(); //get the right wheel encoder

      data_emitted = malloc(5 * sizeof(double)); //assign memory for the 5 counters to send back
      
      memcpy(data_emitted, fitness, 5 * sizeof(double));
      
      //Send the reward/punishment counters to the supervisor
      wb_emitter_send(emitter, data_emitted, 5 * sizeof(double));
      emitter_counter = steps;
      
     //reset the wheel encoders and sum
      for (int i = 0; i < NUM_SENSORS; i++){
        sensor_values[i] = 0.0;
      }
      
      //Reset the wheel encoders, counters, etc for next run 
      wb_differential_wheels_set_encoders (0.0,0.0);
      left = 0; 
      right = 0; 
      old_left = 0; 
      old_right = 0;
      ctx_leftSpeed = 0;
      ctx_rightSpeed = 0;
     fitness[0]= 0; 
     fitness[1] =0;
     fitness[2] =0;
     fitness[3] =0; 
     fitness[4] =0; 
      
    }
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
