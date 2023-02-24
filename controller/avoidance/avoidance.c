#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/console.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/utils/system.h>

#define MAX_SPEED 100
#define TIME_STEP 32

int main()
{
  int range_finder_width;

  wb_robot_init();

  WbDeviceTag range_finder = wb_robot_get_device("range finder");
  wb_range_finder_enable(range_finder, TIME_STEP);

  range_finder_width = wb_range_finder_get_width(range_finder);

  WbDeviceTag left_motor = wb_robot_get_device("left_motor");
  WbDeviceTag right_motor = wb_robot_get_device("right_motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  float left_speed = MAX_SPEED;
  float right_speed = MAX_SPEED;

  while (wb_robot_step(TIME_STEP) != -1)
  {

    // range_finder_width, range_finder and image must be initialized
    const float *image = wb_range_finder_get_range_image(range_finder);

    // starting values for the distances
    float left_distance = 0.1;
    float mid_distance = 0.1;
    float right_distance = 0.1;

    // Constants for the image size
    const int image_height = 40;
    const int image_width = range_finder_width;

    // Width of the left and right parts of the image
    const int left_width = image_width / 3;
    const int right_width = image_width - 2 * left_width;

    // Loop over the image and find the closest object
    for (int x = 0; x < image_width; x++)
    {
      float distance = wb_range_finder_image_get_depth(image, image_width, x, image_height);

      // update the distnace for the left
      if (x < left_width && distance < left_distance)
      {
        left_distance = distance;
      }
      // update the distance for the middle
      else if (x >= left_width && x < left_width + right_width && distance < mid_distance)
      {
        mid_distance = distance;
      }
      // update the distance for the right
      else if (x >= left_width + right_width && distance < right_distance)
      {
        right_distance = distance;
      }
    }
    // front is blocked go back
    if (left_distance < 0.1 && mid_distance < 0.1 && right_distance < 0.1)
    {
      left_speed = -MAX_SPEED * left_distance;
      right_speed = -MAX_SPEED * right_distance;
    }
    // front is open go forward
    else if (left_distance > 0.1 && mid_distance > 0.1 && right_distance > 0.1)
    {
      left_speed = MAX_SPEED * left_distance;
      right_speed = MAX_SPEED * right_distance;
    }
    // left and mid are blocked
    else if (left_distance < 0.1 && mid_distance < 0.1 && right_distance > 0.1)
    {
      left_speed = MAX_SPEED * left_distance;
      right_speed = -MAX_SPEED * right_distance;
    }
    // left and right are blocked turn back
    else if (left_distance < 0.1 && mid_distance > 0.1 && right_distance < 0.1)
    {
      // turn back
      bool turn = false;
      double time1 = wb_robot_get_time(), time2 = 0;
      while (!turn && wb_robot_step(TIME_STEP) != -1)
      {
        left_speed = -MAX_SPEED * left_distance;
        right_speed = MAX_SPEED * right_distance;
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        if (time2 - time1 > 1)
          turn = true;
        time2 = wb_robot_get_time();
      }
    }
    // mid and right are blocked turn left
    else if (left_distance > 0.1 && mid_distance < 0.1 && right_distance < 0.1)
    {
      left_speed = -MAX_SPEED * left_distance;
      right_speed = MAX_SPEED * right_distance;
    }
    // left is blocked turn right
    else if (left_distance < 0.1 && mid_distance > 0.1 && right_distance > 0.1)
    {
      left_speed = MAX_SPEED * left_distance;
      right_speed = -MAX_SPEED * right_distance;
    }
    // mid is blocked turn left
    else if (left_distance > 0.1 && mid_distance < 0.1 && right_distance > 0.1)
    {
      left_speed = -MAX_SPEED * left_distance;
      right_speed = MAX_SPEED * right_distance;
    }
    // right is blocked turn left
    else if (left_distance > 0.1 && mid_distance > 0.1 && right_distance < 0.1)
    {
      left_speed = -MAX_SPEED * left_distance;
      right_speed = MAX_SPEED * right_distance;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}