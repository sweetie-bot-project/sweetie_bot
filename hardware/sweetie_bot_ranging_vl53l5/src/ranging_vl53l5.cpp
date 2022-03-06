#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

//#include "platform/platform.h"

extern "C" {
#include "uld-driver/inc/vl53l5cx_api.h"
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twilight_sparkle");
  ros::NodeHandle n;

  ros::Publisher ranging_pub = n.advertise<sensor_msgs::PointCloud>("ranging", 1);

  ros::Rate loop_rate(10);

  sensor_msgs::PointCloud letter;

  VL53L5CX_Configuration 	Dev;
  VL53L5CX_Configuration* 	p_dev = &Dev;


  /*********************************/
  /*   Power on sensor and init    */
  /*********************************/

  int status_init;
  /* Initialize channel com */
  status_init = vl53l5cx_comms_init(&Dev.platform);
  if(status_init)
  {
      printf("VL53L5CX comms init failed\n");
      return -1;
  }

  /*********************************/
  /*   VL53L5CX ranging variables  */
  /*********************************/

  uint8_t 				status, loop, isAlive, isReady, i;
  uint32_t 				integration_time_ms;
  VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


  /*********************************/
  /*   Power on sensor and init    */
  /*********************************/

  /* (Optional) Check if there is a VL53L5CX sensor connected */
  status = vl53l5cx_is_alive(p_dev, &isAlive);
  if(!isAlive || status)
  {
      printf("VL53L5CX not detected at requested address\n");
      return status;
  }

  /* (Mandatory) Init VL53L5CX sensor */
  status = vl53l5cx_init(p_dev);
  if(status)
  {
      printf("VL53L5CX ULD Loading failed\n");
      return status;
  }

  printf("VL53L5CX ULD ready ! (Version : %s)\n",
          VL53L5CX_API_REVISION);


  /*********************************/
  /*        Set some params        */
  /*********************************/

  /* Set resolution in 8x8. WARNING : As others settings depend to this
   * one, it must be the first to use.
   */
  status = vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);
  if(status)
  {
      printf("vl53l5cx_set_resolution failed, status %u\n", status);
      return status;
  }

  /* Set ranging frequency to 10Hz.
   * Using 4x4, min frequency is 1Hz and max is 60Hz
   * Using 8x8, min frequency is 1Hz and max is 15Hz
   */
  status = vl53l5cx_set_ranging_frequency_hz(p_dev, 10);
  if(status)
  {
      printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
      return status;
  }

  /* Set target order to closest */
  status = vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
  if(status)
  {
      printf("vl53l5cx_set_target_order failed, status %u\n", status);
      return status;
  }

  /* Get current integration time */
  status = vl53l5cx_get_integration_time_ms(p_dev, &integration_time_ms);
  if(status)
  {
      printf("vl53l5cx_get_integration_time_ms failed, status %u\n", status);
      return status;
  }
  printf("Current integration time is : %d ms\n", integration_time_ms);


  /*********************************/
  /*         Ranging loop          */
  /*********************************/

  status = vl53l5cx_start_ranging(p_dev);

  int count = 0;
  loop = 0;
  while (ros::ok())
  {
      /* Use polling function to know when a new measurement is ready.
       * Another way can be to wait for HW interrupt raised on PIN A3
       * (GPIO 1) when a new measurement is ready */

      status = vl53l5cx_check_data_ready(p_dev, &isReady);

      if(isReady)
      {
          vl53l5cx_get_ranging_data(p_dev, &Results);

          /* As the sensor is set in 8x8 mode, we have a total
           * of 64 zones to print. For this example, only the data of
           * first zone are print */
          //printf("Print data no : %3u\n", p_dev->streamcount);
          letter.points.clear();
          for(i = 0; i < 64; i++)
          {
              printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
                  i,
                  Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
                  Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);

              geometry_msgs::Point32 g;
              const float f = 3.5;
              float cx = ((i%8) - 3.5f) / f;
              float cy = ((i/8) - 3.5f) / f;
              float d = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]/1000.0f;
              // g.x = d  / (1 + cx*cx + cy*cy);
              g.x = d;
              g.y = cx * g.x;
              g.z = cy * g.x;
              letter.points.push_back(g); //Results.distance_mm);
          }
          printf("\n");
          loop++;
      }

      /* Wait a few ms to avoid too high polling (function in platform
       * file, not in API) */
      WaitMs(&p_dev->platform, 5);

      letter.header.stamp = ros::Time::now();
      letter.header.frame_id = "map";

      ranging_pub.publish(letter);

      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }
  return 0;
}
