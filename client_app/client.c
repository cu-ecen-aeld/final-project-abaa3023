#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>

#define SERVER_ADDRESS "10.0.0.16"  //Server address to which will be connected.
#define PORT_ADDRESS 8080  //Port address to which data will be sent.
#define MAX_BINARY_READ (size_t)100

char server_ip_address[50];
int socket_desc;    //Socket descriptor.
struct sockaddr_in server;  //Server details.

void configure_server_ip_address(const char *ip_address)
{
    strcpy(&server_ip_address[0], ip_address);
    printf("\n\r IP address of the server is %s", server_ip_address);
}

int init_tcp_connection()
{
    //Create socket.
    socket_desc = socket(AF_INET, SOCK_STREAM, 0);  //Create a socket.
    if(socket_desc == -1){
        printf("Socket couldn't be created!\n");
        printf("%s", strerror(errno));
        exit(EXIT_FAILURE);
    }

    server.sin_addr.s_addr = inet_addr(&server_ip_address[0]); //Specify server IP address to which will be connected.

    server.sin_family = AF_INET;                        //Use IPV4 domain.
    server.sin_port = htons(PORT_ADDRESS);              //Port address to which data will sent.

    //Connect to server.
    if(connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0){   //Connect the server with given configuration.
        printf("Connection error!\n");
        printf("%s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    printf("Connected!\n");

    return 0;
}

void tx_frame(unsigned char *frame, unsigned int file_size)
{

    unsigned int tx_bytes = 0;

    while(file_size){
        tx_bytes = send(socket_desc, frame, file_size, 0);  //Send the data that was written from JPG file
        printf("%d bytes have been sent!\n",tx_bytes);
        file_size -= tx_bytes;    //Decrement file_size
    }

    printf("Image has been sent!\n");
}

