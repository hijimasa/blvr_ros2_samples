/**
* @file blvr_comunicator.h
* @brief header file for blvr motor comunication
* @author Masaaki Hijikata <hijikata@ir.utsunomiya-u.ac.jp>, Utsunomiya Univ.
* @date 20210218
* @details  
*/

extern "C" {
#include <time.h>
#include <termios.h>
}
#include <stdint.h>
#include <string>

#ifndef __BLVR_COMUNICATOR_HPP__
#define __BLVR_COMUNICATOR_HPP__


class BlvrComunicator
{
public:
  static constexpr char BLVRD_DEFAULT_DEVICE[] = "/dev/tty_mortor";
  static const int BLVRD_DEFAULT_FREQUENCY = 10;

  static const int BAUDRATE = B230400;
  static const int BLVR_MAX_RPM = 4000;
  static constexpr double BLVR_RATED_TORQUE = 0.64; /*Nm*/
  static const int BLVR_STEP_TO_ONESHOT = 36000;

  static const int ACCESS_DELAY = 3500; /* us */
  static const int READ_TIMEOUT = 100;

  static const int MSEC = 1000000; //nanosec to msec
  static const int USEC = 1000;    //nanosec to usec

  static constexpr struct timespec BROADCAST_DELAY    = {0,  10 * MSEC};
  static constexpr struct timespec RESPONSE_DELAY     = {0,   5 * MSEC};
  static constexpr struct timespec READ_RETRY_DELAY   = {0,    5 * MSEC};
  static constexpr struct timespec CHARACTER_DELAY    = {0,   10 * USEC};

  static const int MESSAGE_BUF_SIZE = 255;

  enum class return_type : uint8_t
  {
    SUCCESS = 0,
    ERROR = 1
  };
  enum Mode
  {
    NOCONTROL,
    ABSOLUTE_POSITION,
    RELATIVE_POSITION_FROM_TARGET,
    RELATIVE_POSITION_FROM_CURRENT,
    RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_TARGET = 5,
    RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_CURRENT,
    CONTINUOUS_OPERATION_BY_RPM = 16,
    CONTINUOUS_OPERATION_BY_PUSH,
    CONTINUOUS_OPERATION_BY_TORQUE,
    PUSHING_OPERATION_FROM_ABSOLUTE_POSITION = 20,
    PUSHING_OPERATION_FROM_TARGET_POSITION,
    PUSHING_OPERATION_FROM_CURRENT_POSITION
  };

  BlvrComunicator()
  : is_open(false)
  , blvr_port(0) {};
  ~BlvrComunicator() {};

  return_type openDevice(std::string& device);
  void        closeDevice();

  int   directDataDrive(int ch, Mode mode, int position, int rpm, int acc_rate, int dec_rate, int torque);
  int   setExcitation(int ch);

  int   readStep(int ch, int *step);
  int   readRpm(int ch, int *rpm);
  int   readAlarm(int ch, int *alarm);
  int   readWarning(int ch, int *warning);
  int   readTorque(int ch, int *torque);
  int   resetAlarm(int ch);

  bool is_open;

private:
  uint16_t makeCrc16(uint8_t *p, size_t len);

  int blvr_port;
};

#endif /* __BLVR_COMUNICATOR_HPP__ */
