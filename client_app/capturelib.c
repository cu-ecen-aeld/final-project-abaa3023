/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h> 
#include <syslog.h>
#include <stdbool.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include <linux/videodev2.h>

#include <time.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define MAX_HRES (1920)
#define MAX_VRES (1080)
#define MAX_PIXEL_SIZE (3)

#define HRES (640)
#define VRES (480)
#define PIXEL_SIZE (2)
#define HRES_STR "640"
#define VRES_STR "480"

//#define HRES (320)
//#define VRES (240)
//#define PIXEL_SIZE (2)
//#define HRES_STR "320"
//#define VRES_STR "240"

#define STARTUP_FRAMES (30)
#define LAST_FRAMES (1)
#define CAPTURE_FRAMES (90+LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + STARTUP_FRAMES + LAST_FRAMES)

#define FRAMES_PER_SEC (30) 

#define MAX_FRAME_BUFFERS (FRAMES_PER_SEC*3)

#define COLOR_CONVERT_RGB
#define DUMP_FRAMES

#define DRIVER_MMAP_BUFFERS (6)  // request buffers for delay


#define TRUE   (1)
#define FALSE  (0)

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;
struct v4l2_buffer v4l2_buf;

struct buffer 
{
        void   *start;
        size_t  length;
};

static int              camera_device_fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              force_format=1;


static double fnow=0.0, fstart=0.0, fstop=0.0;
static struct timespec time_now, time_start, time_stop;

struct frame_buf_t {
   unsigned char      *frame;
   int                frame_size;
   int                frame_index;
   struct frame_buf_t *next_buf;   
};

/* frame buf pool structure */
struct frame_buf_pool_t {
   struct frame_buf_t *frame_buf;
   void *buf_head;
   int pool_cnt;

   pthread_mutex_t frame_buf_lock;
} frame_buf_pool ;

//circular buffer global variables
#define FRAME_PROCESS_QUEUE  (0)
#define FRAME_TX_QUEUE       (1)

struct queue_struct_t {
   struct frame_buf_t *frame_buf[MAX_FRAME_BUFFERS];
   uint32_t         wptr;              // write pointer
   uint32_t         rptr;              // read pointer
   bool             full;              // full flag
   bool             empty;   // empty flag 

   pthread_mutex_t cb_lock; //circular buffer lock
};
 
struct queue_struct_t   process_queue; // the queue
struct queue_struct_t   tx_queue; // the queue

/*
 * @brief Private function used only by this .c file.
 *
 * Given the pointer of the queue, this function calculates next pointer value.
 *
 * @param ptr (in) - ptr of the messsage queue
 *
 * @return uint32_t (out) - next ptr value
 */
static uint32_t nextPtr(uint32_t ptr) {
	
  if ((ptr + 1) >= MAX_FRAME_BUFFERS) {
      return 0;
  }

  return (ptr+1);
} // nextPtr()

/*
 * refer circular_buffer.h for the documentation
 */
bool write_queue (struct frame_buf_t *frame_buf, uint8_t queue_type) {

  uint32_t nptr; //next ptr
  struct queue_struct_t  *queue;

  if (queue_type == FRAME_PROCESS_QUEUE) {
      queue = &process_queue;
  } else if (queue_type == FRAME_TX_QUEUE) {
      queue = &tx_queue;
  }

  pthread_mutex_lock(&queue->cb_lock);

  //queue full and other error condition
  if (queue->full || (!queue->empty && (queue->wptr == queue->rptr))) {
    pthread_mutex_unlock(&queue->cb_lock); 
    return TRUE;
  }

  /* make sure that wptr is within the queue size limit*/
  if (queue->wptr >= MAX_FRAME_BUFFERS) {
	  pthread_mutex_unlock(&queue->cb_lock);
	  return TRUE;
  }

  //get next pointer value of write ptr
  nptr = nextPtr(queue->wptr);

  queue->frame_buf[queue->wptr] = frame_buf;

  // set empty flag to false
  queue->empty = 0;

  if (nptr == queue->rptr) { // queue full condition
	  queue->full = 1;
  } else {
	  queue->wptr = nptr; // advance the ptr only when queue is not full
  }

  pthread_mutex_unlock(&queue->cb_lock);
  return FALSE;

} // write_queue()


/*
 * refer circular_buffer.h for the documentation
 */
bool read_queue (struct frame_buf_t **frame_buf, uint8_t queue_type) {

	uint32_t nptr; //next ptr
	struct queue_struct_t  *queue;

	if (queue_type == FRAME_PROCESS_QUEUE) {
		queue = &process_queue;
	} else if (queue_type == FRAME_TX_QUEUE) {
		queue = &tx_queue;
	}

	pthread_mutex_lock(&queue->cb_lock);

	//queue empty and other error condition
	if (queue->empty) {
		pthread_mutex_unlock(&queue->cb_lock);
		return TRUE;
	}

	/* make sure that rptr is within the queue size limit*/
	if (queue->rptr >= MAX_FRAME_BUFFERS) {
		pthread_mutex_unlock(&queue->cb_lock);
		return TRUE;
	}

	*frame_buf = queue->frame_buf[queue->rptr];

	//get next pointer value of read ptr
	nptr = nextPtr(queue->rptr);

	if (queue->full) {
		queue->wptr = nextPtr(queue->wptr);
	}

	queue->rptr = nptr; //advance the rptr

	if (queue->wptr == queue->rptr) { // queue empty condition
		queue->empty = 1;
	}

	queue->full = 0; //clear the full flag after successfully reading the q_entry

	pthread_mutex_unlock(&queue->cb_lock);
	return FALSE;

} // read_queue()

/* functions to get/add back frame buffers to free pool*/
struct frame_buf_t* get_frame_buf()
{

	struct frame_buf_t* buf = NULL;
        
	pthread_mutex_lock(&frame_buf_pool.frame_buf_lock);
	if (frame_buf_pool.pool_cnt == 0) {
		//do nothing, return null
	} else { 
            buf = frame_buf_pool.frame_buf; 
            frame_buf_pool.frame_buf = frame_buf_pool.frame_buf->next_buf;
            frame_buf_pool.pool_cnt--; 
        }
        pthread_mutex_unlock(&frame_buf_pool.frame_buf_lock);

	return buf;
}

void enq_frame_buf(struct frame_buf_t* buf)  
{
        pthread_mutex_lock(&frame_buf_pool.frame_buf_lock);
        if (frame_buf_pool.pool_cnt == 0) {
            frame_buf_pool.frame_buf = buf; 
        } else { 
            buf->next_buf = frame_buf_pool.frame_buf;
            frame_buf_pool.frame_buf = buf; 
        }
        frame_buf_pool.pool_cnt++;
        pthread_mutex_unlock(&frame_buf_pool.frame_buf_lock);
}


static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

int init_frame_buffers()
{
   int i = 0;
   void *buf = NULL;
   struct frame_buf_t *frame_buf = NULL;

   /* allocate frame buffers based on frame size*/
    buf = malloc((MAX_FRAME_BUFFERS * HRES*VRES*PIXEL_SIZE) + MAX_FRAME_BUFFERS * sizeof(struct frame_buf_t));
    if (buf == NULL) {
        syslog(LOG_ERR, "init_frame_buffers: failed to allocate memory for frame buffers");
        return -1;
    }

    /* initialize the lock*/
    if (pthread_mutex_init(&frame_buf_pool.frame_buf_lock, NULL) != 0)
    {
        syslog(LOG_ERR, "mutex init failed\n");
        free(buf);

        return -1;
    }

    process_queue.empty= true;
    tx_queue.empty = true;

    /* initialize frame buf pool with default values */
    frame_buf_pool.pool_cnt = 0;
    frame_buf_pool.frame_buf = NULL;
    frame_buf_pool.buf_head = buf; //holds the reference of the pointer

    for (i = 0; i < MAX_FRAME_BUFFERS; i++) {
	frame_buf = buf;
	buf += sizeof(struct frame_buf_t);
	frame_buf->frame = buf;
	buf += (HRES*VRES*PIXEL_SIZE);
        enq_frame_buf(frame_buf);  
    }
    
    syslog(LOG_INFO, "Frame buffers initialized...frame_size=%d", HRES*VRES*PIXEL_SIZE);
    printf("\n\r Frame buffers initialized...frame_size=%d", HRES*VRES*PIXEL_SIZE);
 
    return 0; //SUCCESS
}

// call this function only after all threads are exited
void uninit_frame_buffers()
{
    free(frame_buf_pool.buf_head);
    
    /* initialize frame buf pool with default values */
    frame_buf_pool.frame_buf = NULL;
    frame_buf_pool.pool_cnt = 0;
    frame_buf_pool.frame_buf = NULL;
}

static int xioctl(int fh, int request, void *arg)
{
    int rc;

    do 
    {
        rc = ioctl(fh, request, arg);

    } while (-1 == rc && EINTR == errno);

    return rc;
}


char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="frames/test0000.ppm";

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[11], 9, "%04d", tag);
    strncat(&ppm_dumpname[15], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    
    // subtract 1 from sizeof header because it includes the null terminator for the string
   // written=write(dumpfd, ppm_header, sizeof(ppm_header)-1);
    tx_frame(ppm_header, sizeof(ppm_header)-1);
    tx_frame(p, size);

    total=0;
#if 0
    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);
#endif
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    //printf("Frame written to flash at %lf, %d, bytes\n", (fnow-fstart), total);

    close(dumpfd);
    
}

void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}


// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


// always ignore STARTUP_FRAMES while camera adjusts to lighting, focuses, etc.
int read_framecnt=-STARTUP_FRAMES;
int process_framecnt=0;
int save_framecnt=0;

//unsigned char scratchpad_buffer[FRAMES_PER_SEC][MAX_HRES*MAX_VRES*MAX_PIXEL_SIZE];
unsigned char scratchpad_buffer[MAX_HRES*MAX_VRES*MAX_PIXEL_SIZE];


static int save_image(const void *p, int size, struct timespec *frame_time)
{
    int i, newi, newsize=0;
    unsigned char *frame_ptr = (unsigned char *)p;

    save_framecnt++;
    //printf("save frame %d: ", save_framecnt);
    

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

#if defined(COLOR_CONVERT_RGB)
       
        if(save_framecnt > 0) 
        {
            dump_ppm(frame_ptr, ((size*6)/4), save_framecnt, frame_time);
            //printf("Dump YUYV converted to RGB size %d\n", size);
        }
#endif

    }
    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        //printf("Dump RGB as-is size %d\n", size);
        dump_ppm(frame_ptr, size, process_framecnt, frame_time);
    }
    else
    {
        printf("ERROR - unknown dump format\n");
    }

    return save_framecnt;
}

static int process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *frame_ptr = (unsigned char *)p;

    process_framecnt++;
    printf("process frame %d: ", process_framecnt);
    
    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("NO PROCESSING for graymap as-is size %d\n", size);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {
#if defined(COLOR_CONVERT_RGB)
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        {
            y_temp=(int)frame_ptr[i]; u_temp=(int)frame_ptr[i+1]; y2_temp=(int)frame_ptr[i+2]; v_temp=(int)frame_ptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &scratchpad_buffer[newi], &scratchpad_buffer[newi+1], &scratchpad_buffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &scratchpad_buffer[newi+3], &scratchpad_buffer[newi+4], &scratchpad_buffer[newi+5]);
        }
#endif
    }

    return process_framecnt;
}


static int read_frame(void)
{
    CLEAR(v4l2_buf);

    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(camera_device_fd, VIDIOC_DQBUF, &v4l2_buf))
    {
        switch (errno)
        {
            case EAGAIN:
                return EAGAIN;;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                   non-fatal errors too.
                 */
                return EIO;


            default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }

    read_framecnt++;

    printf("frame %d ", read_framecnt);

    if(read_framecnt == 0) 
    {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

    assert(v4l2_buf.index < n_buffers);

    return 0;
}


int seq_frame_read(void)
{
    fd_set fds;
    struct timeval tv;
    struct frame_buf_t *buf;
    int rc;

    FD_ZERO(&fds);
    FD_SET(camera_device_fd, &fds);

    /* Timeout */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    rc = select(camera_device_fd + 1, &fds, NULL, NULL, &tv);

    buf = get_frame_buf();
    if (buf == NULL) {
       printf("\n\rNo free frame buffs available... try again");
       return -1;
    }

    if (read_frame() != 0) {
	enq_frame_buf(buf);  
	return 0;
    }

    // save off copy of image with time-stamp here
    memcpy((void *)buf->frame, buffers[v4l2_buf.index].start, v4l2_buf.bytesused);

    buf->frame_size = v4l2_buf.bytesused;

    if (buf->frame_size != HRES*VRES*PIXEL_SIZE) {
	printf("Invalid frame size:%d", buf->frame_size);
    }

    if (write_queue (buf, FRAME_PROCESS_QUEUE) != 0) {
	printf("\n\r Unable to enqueue frame to process queue");
    }

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;

    if(read_framecnt > 0)
    {	
        syslog(LOG_CRIT, "read_framecnt=%d at %lf and %lf FPS", read_framecnt, (fnow-fstart), (double)(read_framecnt) / (fnow-fstart));
    }
    else 
    {
        printf("at %lf\n", fnow);
    }

    if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &v4l2_buf))
        errno_exit("VIDIOC_QBUF");
}



int seq_frame_process(void)
{
    int cnt = 0;
    int cur_head_idx, next_head_idx = 0;
    struct frame_buf_t *buf;

    for (;;) {

       if (read_queue(&buf, FRAME_PROCESS_QUEUE) != 0) {
            //printf("\nFRAME_PROCESS: Frames not available yet\n");
            return 0;
        }

        cnt=process_image((void *)buf->frame, buf->frame_size);
        enq_frame_buf(buf);  
#if 0      
        if (write_queue(buf, FRAME_TX_QUEUE) != 0) {
	    printf("\n\r unable to enq frame buffer into TX queue");
	}
#endif
        if(process_framecnt > 0)
        {	
            clock_gettime(CLOCK_MONOTONIC, &time_now);
            fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
            printf(" processed at %lf, @ %lf FPS\n", (fnow-fstart), (double)(process_framecnt+1) / (fnow-fstart));
            save_image(scratchpad_buffer, HRES*VRES*PIXEL_SIZE, &time_now);
 
        }
        else 
        {
            printf("at %lf\n", fnow-fstart);
        }
    }

    return cnt;
}


int seq_frame_store(void)
{
    int cnt;

    return 0;
    cnt=save_image(scratchpad_buffer, HRES*VRES*PIXEL_SIZE, &time_now);
    printf("save_framecnt=%d ", save_framecnt);


    if(save_framecnt > 0)
    {	
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
                printf(" saved at %lf, @ %lf FPS\n", (fnow-fstart), (double)(process_framecnt+1) / (fnow-fstart));
    }
    else 
    {
        printf("at %lf\n", fnow-fstart);
    }

    return cnt;
}


static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    clock_gettime(CLOCK_MONOTONIC, &time_stop);
    fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / 1000000000.0;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == xioctl(camera_device_fd, VIDIOC_STREAMOFF, &type))
		    errno_exit("VIDIOC_STREAMOFF");

    printf("capture stopped\n");
}


static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

	printf("\n\r will capture to %d buffers\n", n_buffers);

        for (i = 0; i < n_buffers; ++i) 
        {
                printf("allocated buffer %d\n", i);

                CLEAR(v4l2_buf);
                v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                v4l2_buf.memory = V4L2_MEMORY_MMAP;
                v4l2_buf.index = i;

                if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &v4l2_buf))
                        errno_exit("VIDIOC_QBUF");
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl(camera_device_fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");

	printf("\n\r Camera is ready to capture...");

}


static void uninit_device(void)
{
        unsigned int i;

        for (i = 0; i < n_buffers; ++i)
                if (-1 == munmap(buffers[i].start, buffers[i].length))
                        errno_exit("munmap");

        free(buffers);
	uninit_frame_buffers();
}


static void init_mmap(char *dev_name)
{
        int    i;
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = DRIVER_MMAP_BUFFERS;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

	printf("init_mmap req.count=%d\n",req.count);

        if (-1 == xioctl(camera_device_fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }
	else
	{
	    printf("Device supports %d mmap buffers\n", req.count);

	    // allocate tracking buffers array for those that are mapped
            buffers = calloc(req.count, sizeof(*buffers));


	    // set up double buffer for frames to be safe with one time malloc her or just declare

	}

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) 
	{
                CLEAR(v4l2_buf);

                v4l2_buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                v4l2_buf.memory      = V4L2_MEMORY_MMAP;
                v4l2_buf.index       = n_buffers;

                if (-1 == xioctl(camera_device_fd, VIDIOC_QUERYBUF, &v4l2_buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = v4l2_buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              v4l2_buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              camera_device_fd, v4l2_buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");

                printf("mappped buffer %d\n", n_buffers);
        }

}


static void init_device(char *dev_name)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(camera_device_fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(camera_device_fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(camera_device_fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one works for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(camera_device_fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(camera_device_fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    init_tcp_connection();
    init_mmap(dev_name);
    init_frame_buffers();
}


static void close_device(void)
{
        if (-1 == close(camera_device_fd))
                errno_exit("close");

        camera_device_fd = -1;
}


static void open_device(char *dev_name)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        camera_device_fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == camera_device_fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}


int v4l2_frame_acquisition_initialization(char *dev_name)
{
    // initialization of V4L2
    open_device(dev_name);
    init_device(dev_name);

    start_capturing();
}


int v4l2_frame_acquisition_shutdown(void)
{
    // shutdown of frame acquisition service
    stop_capturing();

    printf("Total capture time=%lf, for %d frames, %lf FPS\n", (fstop-fstart), read_framecnt+1, ((double)read_framecnt / (fstop-fstart)));

    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}



