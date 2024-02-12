/*
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <byteswap.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <pthread.h>
#include <limits.h>
#include <stdbool.h>

#include <sys/types.h>
#include <sys/mman.h>

/* ltoh: little to host */
/* htol: little to host */
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define ltohl(x)       (x)
#define ltohs(x)       (x)
#define htoll(x)       (x)
#define htols(x)       (x)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define ltohl(x)     __bswap_32(x)
#define ltohs(x)     __bswap_16(x)
#define htoll(x)     __bswap_32(x)
#define htols(x)     __bswap_16(x)
#endif

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE (32*1024UL)
#define MAP_MASK (MAP_SIZE - 1)


// Usable capacity is (QUEUE_CAPACITY-1)
#define QUEUE_CAPACITY 1024

// A structure to represent a queue
// Thread safe (one thread read and another thread write)
struct Queue {
    int front, rear;
    int* array;
};

struct Queue* createQueue() {
    struct Queue* queue = (struct Queue*) malloc(sizeof(struct Queue));
    queue->front = queue->rear = 0;
    queue->array = (int*) malloc(QUEUE_CAPACITY * sizeof(int));
    return queue;
}

int isFull(struct Queue* queue) {
    return ((queue->front-queue->rear+QUEUE_CAPACITY)%QUEUE_CAPACITY == 1);
}

int isEmpty(struct Queue* queue) {
    return (queue->front == queue->rear);
}

void enqueue(struct Queue* queue, int item) {
    if (isFull(queue)) return;
    queue->array[queue->rear] = item;
    queue->rear = (queue->rear + 1)%QUEUE_CAPACITY;
}

int dequeue(struct Queue* queue) {
    if (isEmpty(queue)) return INT_MIN;
    int item = queue->array[queue->front];
    queue->front = (queue->front + 1)%QUEUE_CAPACITY;
    return item;
}

void *getCharThread(void * queue) {
  int ch = -1;
  while(1) {
    ch = getchar();
    enqueue((struct Queue*)queue, ch);
  }
}

static struct termios oldtty;
static int old_fd0_flags;

static void term_exit(void)
{
    tcsetattr (0, TCSANOW, &oldtty);
    fcntl(0, F_SETFL, old_fd0_flags);
}

static void term_init(bool allow_ctrlc)
{
    struct termios tty;

    memset(&tty, 0, sizeof(tty));
    tcgetattr (0, &tty);
    oldtty = tty;
    old_fd0_flags = fcntl(0, F_GETFL);

    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
                          |INLCR|IGNCR|ICRNL|IXON);
    tty.c_oflag |= OPOST;
    tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
    if (!allow_ctrlc)
        tty.c_lflag &= ~ISIG;
    tty.c_cflag &= ~(CSIZE|PARENB);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    tcsetattr (0, TCSANOW, &tty);

    atexit(term_exit);
}

int main(int argc, char **argv)
{
  int fd;
  void *map_base, *virt_addr;
  uint32_t read_result, writeval, addr_result, data_result;
  off_t target;
  char *device;

  /* not enough arguments given? */
  if (argc < 3) {
    fprintf(stderr,
      "\nUsage:\t%s <device> <file> [ncpus]\n"
      "\tdevice  : character device to access\n"
      "\tfile    : nbf data file to write to device\n"
      "\tncpus   : [optional] number of cpus in system (default = 1)\n\n",
      argv[0]);
    exit(1);
  }

  printf("argc = %d\n", argc);

  device = strdup(argv[1]);
  printf("device: %s\n", device);

  int num_cores = 1;
  if (argc == 4) {
    num_cores = atoi(argv[3]);
    printf("Number of cores: %d\n", num_cores);
  }

  // open /dev/xdma0_user
  if ((fd = open(argv[1], O_RDWR | O_SYNC)) == -1)
    FATAL;
  printf("character device %s opened.\n", argv[1]);
  fflush(stdout);

  // map one page
  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (map_base == (void *)-1)
    FATAL;
  printf("Memory mapped at address %p.\n", map_base);
  fflush(stdout);

	// FPGA Host addresses
	// software to fpga nbf command submission (write-only)
  uint32_t * nbf_cmd_ptr = (uint32_t *) (map_base);
	// software to fpga mmio response (write-only)
  uint32_t * mmio_resp_ptr = (uint32_t *) (map_base + 0x004);
	// fpga to software mmio request buffer count (read-only)
  uint32_t * mmio_req_count_ptr = (uint32_t *) (map_base + 0x008);
	// fpga to software mmio request buffer
	// each request is a 32-bit address plus 32-bit data
  uint32_t * mmio_req_ptr = (uint32_t *) (map_base + 0x00C);
  // NBF read
  uint32_t * nbf_resp_count_ptr = (uint32_t *) (map_base + 0x010);
  uint32_t * nbf_resp_ptr = (uint32_t *) (map_base + 0x014);

  // read NBF file given on command line
  // write every NBF file line (32b each) to NBF loader
  // this sends an NBF command of form "cmd_address_data"
  // the order is: data_lo, data_hi, address_lo, address_hi, cmd
  // data is 64b total, address is 64b and cmd is 8b
  FILE *fp;
  char str[16];
  char* filename = argv[2];

  fp = fopen(filename, "r");
  if (fp == NULL){
      printf("Could not open file %s\n",filename);
      return 1;
  }

  int count = 0;
  while (fgets(str, 16, fp) != NULL) {
    uint32_t word = (uint32_t)strtol(str, NULL, 16);
    //printf("%08x\n", word);
    //printf("%s", str);
    *nbf_cmd_ptr = htoll(word);
    if (count == 4 && word == 0x12) {
      uint32_t resp_cnt = 0;
      do {
        resp_cnt = *nbf_resp_count_ptr;
      } while (resp_cnt < 1);
      uint32_t resp = *nbf_resp_ptr;
      printf("NBF read: %08x\n", resp);
      count = 0;
    } else if (count == 4) {
      count = 0;
    } else {
      count++;
    }
  }
  fclose(fp);
  printf("NBF load complete: %s\n",filename);

  int counter = 0;

  struct Queue* queue = createQueue();
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, getCharThread, (void *)queue);

  term_init(1);

  while (1){
    // check if empty by reading the mmio request buffer size
    read_result = *mmio_req_count_ptr;
    read_result = ltohl(read_result);

    // reading from BP MMIO requires two entries in the buffer
    // bp_stream_mmio converts every BP write or read into an [address, data] pair
    // writes do not send a response to BP
    // reads respond with 64b of data in two writes
    // requires that every BP MMIO address is read-only or write-only
    if (read_result >= 2){
        // read addr and data
        addr_result = *mmio_req_ptr;
        addr_result = ltohl(addr_result);
        data_result = *mmio_req_ptr;
        data_result = ltohl(data_result);

        // BP putchar to Host
        if ((addr_result>>12) == 0x101){
            printf("%c", data_result);
            fflush(stdout);
        }
        // BP finish to host
        else if ((addr_result>>12) == 0x102) {
            uint32_t core_id = (addr_result & 0x00000FFF) >> 3;
            if (data_result == 0){
              printf("[CORE%0x FSH] PASS\n", core_id);
              printf("%0x\n", data_result);
            } else {
              printf("[CORE%0x FSH] FAIL\n", core_id);
              printf("%0x\n", data_result);
            }
            fflush(stdout);
            counter++;
            if (counter == num_cores){
                break;
            }
        }
        // BP getchar from host
        // returns 32b of data
        else if ((addr_result>>12) == 0x100) {
            if (isEmpty(queue)) {
                *mmio_resp_ptr = htoll((uint32_t)0xFFFFFFFF);
            } else {
                *mmio_resp_ptr = htoll((uint32_t)(dequeue(queue)));
            }
        }
        // bad request
        else {
            printf("Unhandled BP MMIO address %0x\n", addr_result);
            // TODO: returning 0 seems dangerous. Might need to indicate if
            // request is read or write from BP
            // Return 0 for spurious requests
            *mmio_resp_ptr = htoll((uint32_t)0x00000000);
        }
    }
  }

  term_exit();

  fflush(stdout);

  if (munmap(map_base, MAP_SIZE) == -1)
    FATAL;
  close(fd);

  return 0;
}
