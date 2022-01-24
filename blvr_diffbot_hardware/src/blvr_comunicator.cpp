/**
* @file blvd.c
* @brief source file for blvd
* @author Chris Takahashi <takahashi@ieat-fresh.com>, i-eat Co., Ltd.
* @date 20210218
* @details 
* RS485(Modbus) command I/F for the BLV series, Oriental Motor Co.Ltd.
*/

extern "C" {
#include <stdio.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <string.h>
}
#include <string>

#include "blvr_diffbot_hardware/blvr_comunicator.h"

constexpr struct timespec BlvrComunicator::BROADCAST_DELAY;
constexpr struct timespec BlvrComunicator::RESPONSE_DELAY;
constexpr struct timespec BlvrComunicator::READ_RETRY_DELAY;
constexpr struct timespec BlvrComunicator::CHARACTER_DELAY;

uint16_t
BlvrComunicator::makeCrc16(uint8_t *p, size_t len)
{
  uint16_t crc = 0xffff;

  for (size_t i = 0; i < len; i++) {
    crc = crc ^ (uint16_t)p[i];
    for (size_t read_status = 0; read_status < 8; read_status++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0xa001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/**
 * @fn open
 * @param [in] *device
 *        device name
 * @return status
 * @retval 0 success
 * @retval -1 failure
 */
BlvrComunicator::return_type
BlvrComunicator::openDevice(std::string& device)
{
  if ((blvr_port = open(device.c_str(), O_RDWR | O_NONBLOCK)) < 0) {
    fprintf(stderr, "%s: can't open port(%s)\n", __func__, device.c_str());
    return return_type::ERROR;
  }

  struct termios p;
  tcgetattr(blvr_port, &p);

  memset(&p, 0, sizeof (struct termios));
  p.c_iflag = IGNPAR;
  p.c_cflag = BAUDRATE | CS8 | CREAD | CLOCAL | PARENB;
  p.c_lflag = p.c_lflag & ~ICANON;

  tcsetattr(blvr_port, TCSAFLUSH, const_cast<const termios*>(&p));

  is_open = true;

  return return_type::SUCCESS;
}

/**
 * @fn close
 * @return status
 * @retval 0 success
 * @retval -1 failure
 */
void
BlvrComunicator::closeDevice()
{
  close(blvr_port);
  is_open = false;
}

/**
 * @fn blvrDirectoryDataDrive
 * @
 */
int
BlvrComunicator::directDataDrive(int ch, Mode mode, int position, int rpm, int acc_rate, int dec_rate, int torque)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE];

  unsigned short query_crc, response_crc;
  int write_status, read_status, communicate_check;
  int wait_counter;
  unsigned char c;
  int response_length;
  int flush_status = 0;


  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  /*
  if(blvr.acc_rate == 0 || blvr.dec_rate == 0)
  {
    acc_time = 1000000000;
    dec_time = 1000000000;
  }
  else
  {
    acc_time = ( fabsf(blvr.target_rpm - blvr.current_rpm) / blvr.acc_rate ) * 1000.0f;
    dec_time = ( fabsf(blvr.target_rpm - blvr.current_rpm) / blvr.dec_rate ) * 1000.0f;
  }
  if(acc_time == 0)
  {
    acc_time = 1;
  }
  if(dec_time == 0)
  {
    dec_time = 1;
  }
  */  
  int acc_time = acc_rate, dec_time = dec_rate;
    if(acc_time == 0)
  {
    acc_time = 1;
  }
  if(dec_time == 0)
  {
    dec_time = 1;
  }

  // printf("acc_time = %d, dec_time = %d\n", acc_time, dec_time);
  if(rpm < -BLVR_MAX_RPM)
  {
    rpm = -BLVR_MAX_RPM;
  }
  else if( rpm > BLVR_MAX_RPM)
  {
    rpm = BLVR_MAX_RPM;
  }

  query[0]  = (unsigned char) ch;
  query[1]  = 0x10;
  query[2]  = 0x00;
  query[3]  = 0x5a;
  query[4]  = 0x00;
  query[5]  = 0x0e;
  query[6]  = 0x1c;
  query[7]  = (unsigned char)(mode >> (8 * 3) & 0xff);
  query[8]  = (unsigned char)(mode >> (8 * 2) & 0xff);
  query[9]  = (unsigned char)(mode >> (8 * 1) & 0xff);
  query[10] = (unsigned char)(mode >> (8 * 0) & 0xff);

  query[11] = (unsigned char)(position >> (8 * 3) & 0xff);
  query[12] = (unsigned char)(position >> (8 * 2) & 0xff);
  query[13] = (unsigned char)(position >> (8 * 1) & 0xff);
  query[14] = (unsigned char)(position >> (8 * 0) & 0xff);

  query[15] = (unsigned char)(rpm >> (8 * 3) & 0xff);
  query[16] = (unsigned char)(rpm >> (8 * 2) & 0xff);
  query[17] = (unsigned char)(rpm >> (8 * 1) & 0xff);
  query[18] = (unsigned char)(rpm >> (8 * 0) & 0xff);
  
  query[19] = (unsigned char)(acc_time >> (8 * 3) & 0xff);
  query[20] = (unsigned char)(acc_time >> (8 * 2) & 0xff);
  query[21] = (unsigned char)(acc_time >> (8 * 1) & 0xff);
  query[22] = (unsigned char)(acc_time >> (8 * 0) & 0xff);
  
  query[23] = (unsigned char)(dec_time >> (8 * 3) & 0xff);
  query[24] = (unsigned char)(dec_time >> (8 * 2) & 0xff);
  query[25] = (unsigned char)(dec_time >> (8 * 1) & 0xff);
  query[26] = (unsigned char)(dec_time >> (8 * 0) & 0xff);

  query[27] = (unsigned char)(torque >> (8 * 3) & 0xff);
  query[28] = (unsigned char)(torque >> (8 * 2) & 0xff);
  query[29] = (unsigned char)(torque >> (8 * 1) & 0xff);
  query[30] = (unsigned char)(torque >> (8 * 0) & 0xff);

  query[31] = 0x00;
  query[32] = 0x00;
  query[33] = 0x00;
  query[34] = 0x01;

  query_crc = makeCrc16(query, 35);

  query[35] = (unsigned char)(query_crc & 0xff);  
  query[36] = (unsigned char)(query_crc >> (8 * 1) & 0xff);  

  write_status = write(blvr_port, query, 37);
  
  // printf("write status :%d\n", write_status);
  // for(int i = 0 ; i < 37; i++)
  // {
  //   printf("%02x\n", query[i]);
  // }

  if( write_status < 37 )
  {
    fprintf(stderr,"%s:%s:%d write query is failed (ch: %d)\n",
                    __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if(query[0] == 0x00) // query is broadcast. NO responce
  {
    nanosleep(&BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep(&RESPONSE_DELAY, NULL);

  read_status = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if( read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %d)\n",
                      __FILE__, __func__, __LINE__, ch);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
      wait_counter = 0;
    }
  }
  
  if(response[1] == 0x90)
  {
    response_length = 5;
  }
  else
  {
    response_length = 8;
  }

  while(read_status < response_length)
  {
    if( read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %02x, response[0]: %02x, respomse[1]: %02x)\n",
                      __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter ++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status-2] != (unsigned char)(response_crc & 0xff) ||  response[read_status-1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr,"%s:%s*%d: responce crc error. (crc_low = %02x, responce[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                    __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 * 1) & 0xff), response[read_status-1]);
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Get exception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  communicate_check = 0;
  for(int i = 0; i < read_status-2; i++)
  {
    if( query[i] != response[i])
    {
      communicate_check ++;
    }
  }

  if(communicate_check > 0)
  {
    fprintf(stderr, "%s:%s:%d: communicate was fail\n", __FILE__, __func__, __LINE__);
    fprintf(stderr, "query  responce\n");

    for(int i = 0; i < read_status-2; i++)
    {
      fprintf(stderr, "   %02x        %02x\n", query[i], response[i]);
    }
    return -1;
  }
  nanosleep(&RESPONSE_DELAY, NULL);

  return 0;
}


/**
 * @fn setExcitation
 */
int
BlvrComunicator::setExcitation(int ch)
{
  unsigned char query[13],response[13], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter, response_length, communicate_check;
  int flush_status = 0;

  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  // printf("flush status %d\n",tcflush(blvr_port, TCIFLUSH));

  query[0]  = (unsigned char)ch;
  query[1]  = 0x10;
  query[2]  = 0x00;
  query[3]  = 0x7c;
  query[4]  = 0x00;
  query[5]  = 0x02;
  query[6]  = 0x04;
  query[7]  = 0x00;
  query[8]  = 0x00;
  query[9]  = 0x00;
  query[10] = 0x01;
  
  query_crc = makeCrc16(query, 11);
  query[11] = (unsigned char)(query_crc & 0xff);  
  query[12] = (unsigned char)(query_crc >> (8 * 1) & 0xff);  


  write_status = write(blvr_port, query, 13);
  
  // printf("write status :%d\n", write_status);

  // for(int i = 0 ; i < 13; i++)
  // {
  //   printf("%02x\n", query[i]);
  // }


  if( write_status < 13 )
  {
    fprintf(stderr,"%s:%s:%d: write query is failed (ch: %d)\n",
                    __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if(query[0] == 0x00) // query is broadcast. NO responce
  {
    nanosleep(&BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep(&RESPONSE_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if( read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n",
                      __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }
  
  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
           //read_status, response[0], response[1]);

  if(response[1] == 0x90)
  {
    response_length = 5;
  }
  else
  {
    response_length = 8;
  }

  wait_counter = 0;
  while(read_status < response_length)
  {
    if( read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %02x, response[0]: %02x, respomse[1]: %02x)\n",
                      __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter ++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
 //     printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status-2] != (unsigned char)(response_crc & 0xff) ||  response[read_status-1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr,"%s:%s*%d: responce crc error. (crc_low = %02x, responce[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                    __FILE__, __func__, __LINE__, (unsigned char)(query_crc & 0xff), response[read_status-2], (unsigned char)(query_crc >> (8 * 1) & 0xff), response[read_status-1]);
    return -1;
  }

  if(response[1] == 0x80)
  {
    fprintf(stderr, "%s:%s:%d: Get exception response. (exception code: %02x)\n", __FILE__, __func__, __LINE__, response[1]);
    return -1;
  }

  communicate_check = 0;
  for(int i = 0; i < read_status-2; i++)
  {
    if( query[i] != response[i])
    {
      communicate_check ++;
    }
  }

  if(communicate_check > 0)
  {
    fprintf(stderr, "%s:%s:%d: communicate was fail\n", __FILE__, __func__, __LINE__);
    fprintf(stderr, "query  responce\n");
    for(int i = 0; i < read_status-2; i++)
    {
      fprintf(stderr, "   %02x        %02x\n", query[i], response[i]);
    }
    return -1;
  }

  nanosleep(&RESPONSE_DELAY, NULL);
  return 0;
}

/**
 * @fn readAlarm
 * 
 */
int
BlvrComunicator::readAlarm(int ch, int *alarm)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0xAC;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blvr_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blvr_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blvr_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];

  *alarm = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readStep
 * 
 */
int
BlvrComunicator::readStep(int ch, int *step)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0xCC;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blvr_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blvr_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
     // printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blvr_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *step = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readRpm
 * 
 */
int
BlvrComunicator::readRpm(int ch, int *rpm)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0xCE;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blvr_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blvr_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
     // printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blvr_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *rpm = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readTorque
 * 
 */
int
BlvrComunicator::readTorque(int ch, int *torque)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0xD6;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blvr_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blvr_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blvr_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *torque = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readWarinig
 * 
 */
int
BlvrComunicator::readWarning(int ch, int *warning)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blvr_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0x80;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blvr_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blvr_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blvr_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blvr_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *warning = (high_data << 16) + low_data;

  return 0;

}

