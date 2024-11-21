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

    options.c_cflag &= ~PARENB; // нет чётности
    options.c_cflag &= ~CSTOPB; // 1 стоп бит
    options.c_cflag &= ~CSIZE; // Очищаем маску размера данных
    options.c_cflag |= CS8; // 8 бит

    options.c_cflag &= ~CRTSCTS;  // апаратное управление потоком
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // программное управление потоком

    options.c_cflag |= (CLOCAL | CREAD); // cread включает возможность чтения через uart // clocal отключает линии, которые сейчас не нужны

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
    int uart_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK/*O_NDELAY*/); // O_RDWR - открыт на чтение и запись O_NOCTTY не делать управ терминалом O_NDELAY работа в неблокирующем режиме, чтоб процесс не стоял на сенд/рессив . O_NONBLOCK для рида?
    if (uart_fd == -1) {
        perror("Error port open");
        return -1;
    }

    int res = configure_uart(uart_fd);
    
//===========================SAME_PART_END===========================\\

    // receive
    char buffer[256];
    int flag = 1;
    while(flag){
        int bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("Got: %s\n", buffer);
        } else if (bytes_read < 0) {
            perror("Error in transmission");
            flag = 0;
        }

         sleep(1); // ?
    }

    // Закрываем порт
    close(uart_fd);

    return 0;
}
