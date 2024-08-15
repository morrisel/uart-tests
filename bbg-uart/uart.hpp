#ifndef UART_H
#define UART_H

#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <poll.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

class uart
{
public:
    enum BaudRate
    {
        BAUD_2400 = B2400,
        BAUD_9600 = B9600,
        BAUD_19200 = B19200,
        BAUD_38400 = B38400,
        BAUD_57600 = B57600
    };

    uart(std::string path, BaudRate BaudRate, char parity, uint32_t databits, uint32_t stopbits,
         uint32_t vmin, uint32_t timeout);
    ~uart();

    bool isOpen();
    ssize_t writeData(const std::string& buff);
    ssize_t readData(std::string &buff, uint32_t sizeRead);

    // GPIO methods
    bool setupGPIO(int pin, const std::string& direction);
    bool readGPIO(int pin, int &value);
    bool writeGPIO(int pin, int value);

private:
    BaudRate m_baudRate;
    char m_parity;
    uint32_t m_databits;
    uint32_t m_stopbits;
    uint32_t m_vmin;
    struct termios m_uart;
    
    uint32_t m_timeout;

    int m_fd;
    bool m_isOpen;

    bool openDevice(const std::string& path);
    bool closeDevice();
    bool initDevice();

    std::string gpioPath(int pin);
};

#endif
