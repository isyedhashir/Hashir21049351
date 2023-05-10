#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <cmath>

#define TIME_STEP 64

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }

  Camera *cm;
  cm = robot->getCamera("CAM");
  cm->enable(TIME_STEP);
  cm->recognitionEnable(TIME_STEP);

  GPS *gps;
  gps = robot->getGPS("gps");
  gps->enable(TIME_STEP);

  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  int avoidObstacleCounter = 0;

  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = -1.0;
    double rightSpeed = -1.0;
    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      leftSpeed = 1.0;
      rightSpeed = -1.0;
    } else { // read sensors
      for (int i = 0; i < 2; i++) {
        if (ds[i]->getValue() < 950.0)
          avoidObstacleCounter = 100;
      }
    }
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);

    // Read GPS data
    const double *gps_data = gps->getValues();
    double latitude_degrees = gps_data[0] * 180.0 / M_PI;
    double longitude_degrees = gps_data[1] * 180.0 / M_PI;
    double altitude_meters = gps_data[2];

    // Print GPS data to the console
    printf("Latitude: %f degrees, Longitude: %f degrees, Altitude: %f meters\n", latitude_degrees, longitude_degrees, altitude_meters);
  }

  delete robot;
  return 0;
}
