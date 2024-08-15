#include "uart.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    try
    {
        // uart initialize
        uart my_uart("/dev/ttyS1", uart::BAUD_9600, 'N', 8, 1, 1, 1);
        std::cout << "UART initialized successfully!" << std::endl;

        // settings
        // For transmit:
        // /sys/class/gpio/gpio72/direction /sys/class/gpio/gpio72/value
        // direction need to configure to: out
        // the value need to configure to: 1
        //
        // For receive:
        // /sys/class/gpio/gpio71/direction /sys/class/gpio/gpio71/value
        // direction need to configure to: in
        // the value need to configure to: 0
        //                                 ^
        //                                 |
        //                    when the connection closed between pinouts the value
        //                    the main.c change it to 1
        
        
        // setting gpio
        my_uart.setupGPIO(71, "in");
        my_uart.setupGPIO(72, "out");

        std::cout << "GPIO 71 set to input mode." << std::endl;
        std::cout << "GPIO 72 set to output mode." << std::endl;

        // data that I want to send
        std::string message = "Hello, World!";
        while (true)
        {
            int value;
            if (my_uart.readGPIO(71, value))
            {
                std::cout << "GPIO 71 value read: " << value << std::endl;
                if (value == 1)
                {
                    std::cout << "GPIO is high. Sending message: " << message << std::endl;
                    ssize_t bytesWritten = 	my_uart.writeData(message);
                    std::cout << "Bytes sent: " << bytesWritten << std::endl;
                }
            }
            else
            {
                std::cerr << "Failed to read GPIO 71 value." << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;

    }

    return 0;
}
