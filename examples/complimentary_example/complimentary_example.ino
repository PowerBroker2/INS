#include "wmm.h"
#include "Open_Controller.h"
#include "INS.h"




using namespace bfs;




sensor_cal     accelCal;
sensor_cal     gyroCal;
sensor_cal     magCal;
OpenController myCtrlr;
compFilt       ahrs;
WmmData        earthMagData;




void setup()
{
  Serial.begin(115200);
  Serial6.begin(115200);

  myCtrlr.begin(Serial, false);

  accelCal.lpf_cutoff_hz = 25;
  myCtrlr.setAccelCal(accelCal);

  gyroCal.intrinsic_bias_vec << -5.71, 3.03, 4.72;
  gyroCal.lpf_cutoff_hz = 25;
  myCtrlr.setGyroCal(gyroCal);

//  magCal.intrinsic_cal_mat << 0.02819196,  0.0011868,  -0.00284967,
//                              0.0011868,   0.03924221,  0.00030842,
//                             -0.00284967,  0.00030842,  0.03737559;
  magCal.intrinsic_bias_vec << 4.691612, -12.9932745, 6.313944;
  magCal.lpf_cutoff_hz = 25;
  myCtrlr.setMagCal(magCal);

  ahrs.begin(0.10);

  earthMagData = wrldmagm(185,
                          43.084510,
                          -75.383543,
                          2023.5);
  Vector3d earthMag;
  earthMag << earthMagData.mag_field_nt[0],
              earthMagData.mag_field_nt[1],
              earthMagData.mag_field_nt[2];
  ahrs.setMag(earthMag);
}




void loop()
{
  myCtrlr.readIMU();
  
  Vector3d a = myCtrlr.getCalAccel();
  Vector3d g = myCtrlr.getCalGyro();
  Vector3d m = myCtrlr.getCalMag();
  Vector3d a_in;
  a_in << 0, 0, 0;

  ahrs.update(a, g, m, -1, a_in);

//  Serial.print("ax:"); Serial.print(a(0));
//  Serial.print(",ay:"); Serial.print(a(1));
//  Serial.print(",az:"); Serial.print(a(2));
//  Serial.println();

//  Serial.print("gx:"); Serial.print(g(0));
//  Serial.print(",gy:"); Serial.print(g(1));
//  Serial.print(",gz:"); Serial.print(g(2));
//  Serial.println();

//  Serial.print("mx:"); Serial.print(m(0));
//  Serial.print(",my:"); Serial.print(m(1));
//  Serial.print(",mz:"); Serial.print(m(2));
//  Serial.println();

  Serial.print("Pitch:"); Serial.print(ahrs.getPitch());
  Serial.print(",Roll:"); Serial.print(ahrs.getRoll());
  Serial.print(",Yaw:");  Serial.println(ahrs.getYaw());

  delay(1);
}