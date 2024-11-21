#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>


int configure_uart(int uart_fd) {
    struct termios options;
    if (tcgetattr(uart_fd, &options) != 0) {
        perror("Error in port settings");
        close(uart_fd);
        return -1;
    }

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    options.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(uart_fd, TCSANOW, &options) != 0) {
        perror("Error in settings");
        close(uart_fd);
        return -1;
    }
    return 1;
}

int main() {
    
//==========================SAME_PART_START==========================\\

    // Open port
    int uart_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("Error port open");
        return -1;
    }

    int res = configure_uart(uart_fd);

//===========================SAME_PART_END===========================\\

    // send
    const char *data = "Test transmission\n";
    int bytes_written = write(uart_fd, data, strlen(data));
    if (bytes_written < 0) {
        perror("Error in transmission");
    }

    // Закрываем порт
    close(uart_fd);

    return 0;
}
