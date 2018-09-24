#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//https://github.com/amaork/libi2c

#define CO2_ADDR 0x15        // default I2C slave address
#define CO2_DEV "/dev/i2c-1" // default I2C device file

int main(void)
{

    int file;
    std::string filename = "/dev/i2c-1";
    int ppmReading;

    if ((file = open(filename.c_str(), O_RDWR)) < 0)
    {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, CO2_ADDR) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    unsigned int buffer_write[8] = {0x15, 0x04, 0x13, 0x8b, 0x00, 0x01, 0x46, 0x70};
    unsigned int buffer_read[7];

    while (1)
    {
        // Using I2C Read
        if (read(file, buffer_write, 2) != 2)
        {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
        }
        else
        {
            write(file, buffer_write, (sizeof(buffer_write)/sizeof(uint8_t)));
            usleep(10000);
            read(file, buffer_read, 8);

            printf("buffer_read value:%X ,%X ,%X ,%X ,%X ,%X ,%X \n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3], buffer_read[4], buffer_read[5], buffer_read[6]);

            ppmReading = buffer_read[3] << 8 | buffer_read[4];

            printf("C02 ppm: %d \n", ppmReading);
        }
        usleep(500000);
        printf("*****\n");
    }

    //unsigned char reg = 0x10; // Device register to access
    //buf[0] = reg;
    //buf[0] = 0b11110000;

    if (write(file, buffer_write, 1) != 1)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
        //buffer = g_strerror(errno);
        //printf(buffer);
        //printf("\n\n");
    }
    return EXIT_SUCCESS;
}
