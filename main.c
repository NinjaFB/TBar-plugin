#include <stdio.h>
#include <string.h>

#ifdef __gnu_linux__

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#endif

#ifdef _WIN64

#include <windows.h>

DWORD dwBytesRead = 0;
#endif

int skip = 0;
unsigned char buffer[3];

int read_serial_port(void *serial_port, unsigned char *buffer) {
  #ifdef __gnu_linux__
  int bytes_read = read(*(int *)serial_port, buffer, sizeof(unsigned char));
  if (bytes_read <= 0) {
    perror("Error reading from serial port");
    return -1;
  }
  return bytes_read;
  #endif

  #ifdef _WIN64
  DWORD dwBytesRead;
  BOOL result = ReadFile(serial_port, buffer, 1, &dwBytesRead, NULL);
  if (!result) {
    fprintf(stderr, "Error reading from serial port\n");
    return -1;
  }
  return (int)dwBytesRead;
  #endif
}


int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("Please enter the port to run on (eg. COM6 or /dev/ttyUSB0 )\n");
    return 1;
  }

  #ifdef _WIN64

  /* Connection to serial on windows borrowed from
   * https://ds.opdenbrouw.nl/micprg/pdf/serial-win.pdf
   */

  HANDLE serial_port;

  serial_port = CreateFile(argv[1],
                       GENERIC_READ | GENERIC_WRITE,
                       0,
                       0,
                       OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL,
                       0);

  if(serial_port==INVALID_HANDLE_VALUE){
    if(GetLastError()==ERROR_FILE_NOT_FOUND){
      printf("Serial Port \"COM6\" does not exist.\n");
    }
    printf("An error occured\n");
  }

  DCB dcbSerialParams = {0};

  dcbSerialParams.DCBlength=sizeof(dcbSerialParams);

  if (!GetCommState(serial_port, &dcbSerialParams)) {
    printf("Error getting state\n");
  }

  dcbSerialParams.BaudRate=CBR_9600;
  dcbSerialParams.ByteSize=8;
  dcbSerialParams.StopBits=ONESTOPBIT;
  dcbSerialParams.Parity=NOPARITY;

  if(!SetCommState(serial_port, &dcbSerialParams)) {
    printf("Error setting state\n");
  }

  #endif

  #ifdef __gnu_linux__

  /* Connection to serial on linux borrowed from
   * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
   */

  int serial_port = open(argv[1], O_RDWR);
  if (serial_port < 0) {
    printf("Error %i from open %s\n", errno, strerror(errno));
  }

  struct termios tty;

  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  #endif

  while (1) {
    read_serial_port(&serial_port, &buffer[0]);
  skip:
    if (buffer[0] == 0xff) {
      read_serial_port(&serial_port, &buffer[1]);
      if (buffer[1] < 0xff) {
        read_serial_port(&serial_port, &buffer[2]);
        printf("%d\n",(unsigned short)((buffer[2] << 8) | buffer[1]));
      } else {
        buffer[0] = buffer[1];
        goto skip;
      }
    }
  }

  #ifdef __gnu_linux__
  close(serial_port);
  #endif
  #ifdef _WIN64
  CloseHandle(serial_port);
  #endif

  return 0;
}
