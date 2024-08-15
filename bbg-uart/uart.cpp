#include "uart.hpp"
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <sstream>
#include <sys/stat.h>

uart::uart(std::string path, BaudRate BaudRate, char parity, uint32_t databits, uint32_t stopbits,
           uint32_t vmin, uint32_t timeout)
{
    m_baudRate = BaudRate;
    m_parity = parity;
    m_databits = databits;
    m_stopbits = stopbits;
    m_vmin = vmin;
    m_timeout = timeout;

    m_fd = -1;
    m_isOpen = false;

    if (!openDevice(path))
    {
        throw std::runtime_error("Failed to open port");
    }

    m_isOpen = true;

    if (!initDevice())
    {
        closeDevice();
        throw std::runtime_error("Failed to initialize uart");
    }
}

uart::~uart()
{
    closeDevice();
}

bool uart::isOpen()
{
    return m_isOpen;
}

ssize_t uart::writeData(const std::string& buff)
{
    return write(m_fd, buff.c_str(), buff.length());
}

ssize_t uart::readData(std::string &buff, uint32_t sizeRead)
{
    buff.resize(sizeRead);
    return read(m_fd, &buff[0], sizeRead);
}

bool uart::openDevice(const std::string& path)
{
    m_fd = open(path.c_str(), O_RDWR | O_NOCTTY);
    if (m_fd == -1)
    {
        return false;
    }

    m_isOpen = true;
    return true;
}

bool uart::closeDevice()
{
    if (isOpen() && close(m_fd) == 0)
    {
        m_isOpen = false;
        return true;
    }
    return false;
}

bool uart::initDevice()
{
    if (tcgetattr(m_fd, &m_uart))
    {
        return false;
    }

    cfsetispeed(&m_uart, m_baudRate);
    cfsetospeed(&m_uart, m_baudRate);

    m_uart.c_cflag &= ~PARENB;     // Clear parity enable
    m_uart.c_cflag &= ~CSTOP;      // Clear stop bits
    m_uart.c_cflag &= ~CSIZE;      // Clear character size bits

    m_uart.c_iflag = IGNBRK;       // 
    m_uart.c_lflag = 0;            // 
    m_uart.c_oflag = 0;            // 

    m_uart.c_cflag |= ~PARENB;     // Set parity bits
    m_uart.c_cflag |= ~CSTOP;      // Set stop bits
    m_uart.c_cflag |= ~CSIZE;      // Set data bits

    m_uart.c_iflag &= ~(IXON | IXOFF | IXANY);

    m_uart.c_cc[VTIME] = m_timeout;
    m_uart.c_cc[VMIN] = m_vmin;

    m_uart.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(m_fd, TCSANOW, &m_uart))
    {
        return false;
    }

    return true;
}

bool uart::setupGPIO(int pin, const std::string& direction)
{
    std::string path = gpioPath(pin) + "/direction";
    std::ofstream gpio_dir(path);
    if (!gpio_dir.is_open())
        return false;
    gpio_dir << direction;
    gpio_dir.close();
    return true;
}

bool uart::readGPIO(int pin, int &value)
{
    std::string path = gpioPath(pin) + "/value";
    std::ifstream gpio_value(path);
    if (!gpio_value.is_open())
        return false;
    gpio_value >> value;
    gpio_value.close();
    return true;
}

bool uart::writeGPIO(int pin, int value)
{
    std::string path = gpioPath(pin) + "/value";
    std::ofstream gpio_value(path);
    if (!gpio_value.is_open())
        return false;
    gpio_value << value;
    gpio_value.close();
    return true;
}

std::string uart::gpioPath(int pin)
{
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << pin;
    return ss.str();
}
