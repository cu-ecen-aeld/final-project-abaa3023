#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>              /* low-level i/o */


#define MAX 80
#define PORT 8080
#define SA struct sockaddr

#define HRES_STR "640"
#define VRES_STR "480"

#define IMAGE_HEADER_LENGTH       (48)
#define RAW_RGB_640_480_SIZE      (921600)
#define RGB_640_480_SIZE          (RAW_RGB_640_480_SIZE+IMAGE_HEADER_LENGTH)
  
unsigned char buf[RGB_640_480_SIZE*2];

char ppm_writename[50]="write_pics/test0000.ppm";
char ppm_header[IMAGE_HEADER_LENGTH];
//="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";

enum state {
    STATE_HEADER,
    STATE_FRAME
};

enum state next_state = STATE_HEADER;
int need_new_file = 1;

int partial_frame_header_offset = 0;
int partial_frame_offset = 0;

unsigned char frame_buffer[RAW_RGB_640_480_SIZE];

int tag = 1;
int dump_fd = 0;
    
// Function to create new file
void create_new_image_file()
{
    int n;
    int excess_bytes = 0, total_bytes = 0, read_bytes = 0,total = 0;
    int written = 0;
    
    snprintf(&ppm_writename[15], 5, "%04d", tag++);
    strncat(&ppm_writename[19], ".ppm", 5);
    dump_fd = open(ppm_writename, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);
    
    need_new_file = 0;
}
 
void frame_state_machine(unsigned char *data, int size)
{
    switch(next_state) {
       case STATE_HEADER:
         if ((size + partial_frame_header_offset) == IMAGE_HEADER_LENGTH) {
             memcpy(&ppm_header[partial_frame_header_offset], data, size);
             printf("\n\r Saving complete header, partial_frame_header_offset=%d", partial_frame_header_offset);
             if (write(dump_fd, ppm_header, sizeof(ppm_header)) != IMAGE_HEADER_LENGTH) {
                 printf("\n\r HEADER_EXACT: Failed to write ppm header");
             }
             partial_frame_header_offset = 0;
             next_state = STATE_FRAME;
         } else if ((size + partial_frame_header_offset) > IMAGE_HEADER_LENGTH) {
             memcpy(&ppm_header[partial_frame_header_offset], data, (IMAGE_HEADER_LENGTH-partial_frame_header_offset));
             printf("\n\r extra header: Saving header, partial_frame_header_offset=%d", partial_frame_header_offset);
             size -= (IMAGE_HEADER_LENGTH - partial_frame_header_offset);
             data += (IMAGE_HEADER_LENGTH - partial_frame_header_offset);
             if (write(dump_fd, ppm_header, sizeof(ppm_header)) != IMAGE_HEADER_LENGTH) {
                 printf("\n\r HEADER_EXTRA: Failed to write ppm header");
             }
             partial_frame_header_offset = 0;
             next_state = STATE_FRAME;
             frame_state_machine(data, size); //save the remaining data which is a frame
         } else {
             printf("\n\r partial header: Saving partial header, partial_frame_header_offset=%d", partial_frame_header_offset);
             memcpy(&ppm_header[partial_frame_header_offset], data, size);
             partial_frame_header_offset += size;
         }
	 break;
       case STATE_FRAME:
         if ((size + partial_frame_offset) == RAW_RGB_640_480_SIZE) {
             memcpy(&frame_buffer[partial_frame_offset], data, size);
             printf("\n\r Saving complete frame, partial_frame_offset=%d", partial_frame_offset);
             if (write(dump_fd, frame_buffer, RAW_RGB_640_480_SIZE) != RAW_RGB_640_480_SIZE) {
                printf("\n\r EXACT_FRAME: write mismatch");
	     }
             partial_frame_offset = 0;
             next_state = STATE_HEADER;
             close(dump_fd);
             create_new_image_file();
         } else if ((size + partial_frame_offset) > RAW_RGB_640_480_SIZE) {
             memcpy(&frame_buffer[partial_frame_offset], data, (RAW_RGB_640_480_SIZE-partial_frame_offset));
             printf("\n\r Extra frame: Saving complete frame, partial_frame_offset=%d", partial_frame_offset);

             size -= (RAW_RGB_640_480_SIZE - partial_frame_offset);
             data += (RAW_RGB_640_480_SIZE - partial_frame_offset);
              if (write(dump_fd, frame_buffer, RAW_RGB_640_480_SIZE) != RAW_RGB_640_480_SIZE) {
                printf("\n\r EXACT_FRAME: write mismatch");
	     }
             partial_frame_offset = 0;
             next_state = STATE_HEADER;
             close(dump_fd);
             create_new_image_file();
             frame_state_machine(data, size); //save the remaining data which is a header
         } else {
             printf("\n\r Partial frame: Saving partial frame, partial_frame_offset=%d", partial_frame_offset);
             memcpy(&frame_buffer[partial_frame_offset], data, size);
             partial_frame_offset += size;
         }
	 break;
       default:
         printf("\n\r State machine shouldn't be coming here...");
         break;
    }
}

#ifdef TEST
unsigned char test_buf[RGB_640_480_SIZE*2];
int main() {
   
    int i = 0;

    printf("\n\r TEST 1:");

    for (i = 0; i < 2; i++) {    
      frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH);
      frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE); 
    }

    printf("\n\r TEST 2:");
    
    for (i = 0; i < 2; i++) {    
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH/2);
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH/2);
 
       frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE/2); 
       frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE/2); 
    }

    printf("\n\r TEST 3:");
    
    for (i = 0; i < 2; i++) {    
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH + RAW_RGB_640_480_SIZE/2);
       frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE/2); 
    }

    printf("\n\r TEST 4:");
    
    for (i = 0; i < 2; i++) {    
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH + RAW_RGB_640_480_SIZE/2);
       frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE/4);
       frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE/4);
    }

    printf("\n\r TEST 5:");
    
    for (i = 0; i < 2; i++) {    
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH/2);
       frame_state_machine(&test_buf, (IMAGE_HEADER_LENGTH/2) + RAW_RGB_640_480_SIZE);
    }

    printf("\n\r TEST 6:");
    
    for (i = 0; i < 2; i++) {    
       frame_state_machine(&test_buf, (IMAGE_HEADER_LENGTH*2) + (RAW_RGB_640_480_SIZE*2));
    }

    printf("\n\r TEST 7:");
    
    for (i = 0; i < 2; i++) {    
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH);
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH/2 + RAW_RGB_640_480_SIZE);
       frame_state_machine(&test_buf, IMAGE_HEADER_LENGTH/2 + RAW_RGB_640_480_SIZE/2);
       frame_state_machine(&test_buf, RAW_RGB_640_480_SIZE/2);
    }



    return 0;
}
#else


// Function designed for chat between client and server.
void func(int sockfd)
{
    int read_bytes = 0;

    if (need_new_file){  //onlly first file is created here
           create_new_image_file(); 
        }

    // infinite loop for chat
    for (;;) {
        for (int i = 0; i < 49; i++)
            printf("%c", buf[i]); 


	read_bytes = read(sockfd, &buf[0], sizeof(buf));
        if (read_bytes < 1) {
               exit(-1);
        }

        printf("\n read_bytes is %d", read_bytes);
        for (int i = 0; i < 49; i++)
            printf("%c", buf[i]); 

        frame_state_machine(&buf[0], read_bytes);

#if 0   
        total_bytes = RGB_640_480_SIZE; 
        while(total_bytes > 0) {  
            read_bytes = read(sockfd, buf, sizeof(buf));
            if (read_bytes < 1) {
               exit(-1);
            }

            // subtract 1 from sizeof header because it includes the null terminator for the string
            written=write(dump_fd, ppm_header, sizeof(ppm_header)-1);

            total_bytes-=read_bytes;

            printf("\n\r written %d bytes and total bytes :%d", read_bytes, total_bytes);
            if (write(dump_fd, &buf, read_bytes) != read_bytes) {
                printf("\n\r read write mismatch");
	    }

        }
  #endif
     }
}
   
// Driver function
int main()
{
    int sockfd, connfd, len;
    struct sockaddr_in servaddr, cli;
   
    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        printf("socket creation failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully created..\n");
    bzero(&servaddr, sizeof(servaddr));
   
    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
   
    // Binding newly created socket to given IP and verification
    if ((bind(sockfd, (SA*)&servaddr, sizeof(servaddr))) != 0) {
        printf("socket bind failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully binded..\n");
   
    // Now server is ready to listen and verification
    if ((listen(sockfd, 5)) != 0) {
        printf("Listen failed...\n");
        exit(0);
    }
    else
        printf("Server listening..\n");
    len = sizeof(cli);
   
    // Accept the data packet from client and verification
    connfd = accept(sockfd, (SA*)&cli, &len);
    if (connfd < 0) {
        printf("server accept failed...\n");
        exit(0);
    }
    else
        printf("server accept the client...\n");
   
    // Function for chatting between client and server
    func(connfd);
   
    // After chatting close the socket
    close(sockfd);
}
#endif
