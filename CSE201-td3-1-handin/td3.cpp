#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
  // IMPLEMENT YOUR FUNCTION HERE
    double *narr;
    narr = new double[new_size];
    for (int i=0; i < new_size; i++){
        if(i < length){
            narr[i] = array[i];
        } else{
            narr[i] = 0;
        }
    }
    delete array;

  return narr; // YOU CAN CHANGE THIS
}

double* shrink_array(double* array, int length, int new_size) {
  // IMPLEMENT YOUR FUNCTION HERE
    double *narr;
    narr = new double[new_size];
    for (int i=0; i < new_size; i++){
        narr[i] = array[i];
    }
    delete array;

  return narr;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
  // IMPLEMENT YOUR FUNCTION HERE
    if (current_size == max_size){
        array = extend_array(array, current_size, current_size +5);
        max_size+=5;
    }
    array[current_size] = element;
    current_size ++;

  return array; // YOU CAN CHANGE THIS
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
  // IMPLEMENT YOUR FUNCTION HERE
    if (current_size  <= max_size-4){
        array = shrink_array(array, max_size, current_size);
        max_size-=5;
    }
    array[current_size] = NULL;
    current_size --;

  return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;
//  int i =telemetry_current_size;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;
//Use already created functions
//  telemetry = append_to_array(0.0,telemetry,telemetry_current_size,telemetry_max_size);
//  telemetry = append_to_array(0.0,telemetry,telemetry_current_size,telemetry_max_size);
//  telemetry = append_to_array(0.0,telemetry,telemetry_current_size,telemetry_max_size);

//  if(telemetry_current_size +3> telemetry_max_size){
//      telemetry = extend_array(telemetry,telemetry_max_size, telemetry_max_size +5);
//  }
//  telemetry[telemetry_current_size] = 0;
//  telemetry[telemetry_current_size+1] = 0;
//  telemetry[telemetry_current_size+2] = 0;
//  telemetry_current_size += 3;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
      telemetry = append_to_array(t,telemetry,telemetry_current_size,telemetry_max_size);
      telemetry = append_to_array(x,telemetry,telemetry_current_size,telemetry_max_size);
      telemetry = append_to_array(y,telemetry,telemetry_current_size,telemetry_max_size);
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
        telemetry = append_to_array(t,telemetry,telemetry_current_size,telemetry_max_size);
        telemetry = append_to_array(x,telemetry,telemetry_current_size,telemetry_max_size);
        telemetry = append_to_array(y,telemetry,telemetry_current_size,telemetry_max_size);
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
      telemetry = append_to_array(t,telemetry,telemetry_current_size,telemetry_max_size);
      telemetry = append_to_array(x,telemetry,telemetry_current_size,telemetry_max_size);
      telemetry = append_to_array(y,telemetry,telemetry_current_size,telemetry_max_size);
//      if(telemetry_current_size +3> telemetry_max_size){
//          telemetry = extend_array(telemetry,telemetry_max_size, telemetry_max_size +5);
//      }
//      telemetry[telemetry_current_size] = t;
//      telemetry[telemetry_current_size+1] = x;
//      telemetry[telemetry_current_size+2] = y;
//      telemetry_current_size+=3;
    }
  }

  return hit_target;
}

void sort(double* &obstacles, const int num_obstacles) {
  // IMPLEMENT YOUR CODE HERE
    int temp;
    int count = 0;
    while(count < num_obstacles-3){
        for (int i = 0; i < num_obstacles - 3;i+=3){
            if(obstacles[i] > obstacles[i+3]){
                //swap
                temp = obstacles[i];
                obstacles[i] = obstacles[i+3];
                obstacles[i+3] = temp;
                //swap the next ones
                temp = obstacles[i+1];
                obstacles[i+1] = obstacles[i+4];
                obstacles[i+4] = temp;
                //swap the next ones
                temp = obstacles[i+2];
                obstacles[i+2] = obstacles[i+5];
                obstacles[i+5] = temp;


                count = 0;
            } else{
                count+=3;
            }

        }
    }
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
    int t = 0;
    int count = 0;
    while(true){ //time
        for(int i = 0; i < tot_telemetries; i++){ //go thru each telemetry
            if(t < telemetries_sizes[i]){
                global_telemetry = append_to_array(telemetries[i][t],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
                global_telemetry = append_to_array(telemetries[i][t+1],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
                global_telemetry = append_to_array(telemetries[i][t+2],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
            } else{
                count++;
            }
        }
        if (count >= tot_telemetries){
            sort(global_telemetry, global_telemetry_current_size);
            break;
        } else{
            count = 0;
            t+=3;
        }

//merge_telemetry([[0, 0, 0, 5, 1, 1], [0, 0, 0, 2, 3, 3, 2.5, 6, 7], [0, 0, 0, 1.2, 1, 1], [0, 0, 0, 3, 1, 1, 10, 20, 20], []], 5, [6, 9, 6, 9, 0]
//[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, 1, 1, 2, 3, 3, 2.5, 6, 7, 3, 1, 1, 5, 1, 1, 10, 20, 20]

    }
}
