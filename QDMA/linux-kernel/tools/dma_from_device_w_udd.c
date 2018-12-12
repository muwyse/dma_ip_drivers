/*
 * This file is part of the QDMA userspace application
 * to enable the user to execute the QDMA functionality
 *
 * Copyright (c) 2018-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#define _DEFAULT_SOURCE
#define _XOPEN_SOURCE 500
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "dma_xfer_utils.c"

#define DEVICE_NAME_DEFAULT "/dev/xdma0_c2h_0"
#define SIZE_DEFAULT (32)
#define COUNT_DEFAULT (1)

static struct option const long_opts[] = {
	{"device", required_argument, NULL, 'd'},
	{"address", required_argument, NULL, 'a'},
    {"size", required_argument, NULL, 's'},
	{"offset", required_argument, NULL, 'o'},
	{"count", required_argument, NULL, 'c'},
    {"file", required_argument, NULL, 'f'},
    {"udd_file", required_argument, NULL, 'u'},
	{"help", no_argument, NULL, 'h'},
	{"verbose", no_argument, NULL, 'v'},
	{0, 0, 0, 0}
};

static int test_dma(char *devname, uint64_t addr, uint64_t size,
		    uint64_t offset, uint64_t count, char *ofname, char *uddofname);
static int no_write = 0;

static void usage(const char *name)
{
	int i = 0;
	fprintf(stdout, "%s\n\n", name);
	fprintf(stdout, "usage: %s [OPTIONS]\n\n", name);
	fprintf(stdout, "Read via SGDMA, optionally save output to a file\n\n");

	fprintf(stdout, "  -%c (--%s) device (defaults to %s)\n",
		long_opts[i].val, long_opts[i].name, DEVICE_NAME_DEFAULT);
	i++;
	fprintf(stdout, "  -%c (--%s) the start address on the AXI bus\n",
	       long_opts[i].val, long_opts[i].name);
	i++;
    fprintf(stdout,
        "  -%c (--%s) size of a single packet in bytes, default %d.\n",
        long_opts[i].val, long_opts[i].name, SIZE_DEFAULT);
    i++;
	fprintf(stdout, "  -%c (--%s) page offset of transfer\n",
		long_opts[i].val, long_opts[i].name);
	i++;
	fprintf(stdout, "  -%c (--%s) number of transfers, default is %d.\n",
	       long_opts[i].val, long_opts[i].name, COUNT_DEFAULT);
    i++;
    fprintf(stdout,
        "  -%c (--%s) file to write the data of the transfers\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout,
        "  -%c (--%s) file to write the UDD data from the transfers\n",
        long_opts[i].val, long_opts[i].name);
	i++;
	fprintf(stdout, "  -%c (--%s) print usage help and exit\n",
		long_opts[i].val, long_opts[i].name);
	i++;
	fprintf(stdout, "  -%c (--%s) verbose output\n",
		long_opts[i].val, long_opts[i].name);
	i++;
}

int main(int argc, char *argv[])
{
	int cmd_opt;
	char *device = DEVICE_NAME_DEFAULT;
	uint64_t address = 0;
	uint64_t size = SIZE_DEFAULT;
	uint64_t offset = 0;
	uint64_t count = COUNT_DEFAULT;
	uint64_t num_pkts = 1;
    char *ofname = NULL;
    char *uddofname = NULL;

	while ((cmd_opt = getopt_long(argc, argv, "vhxc:f:d:a:s:o:u:c:", long_opts,
			    NULL)) != -1) {
		switch (cmd_opt) {
		case 0:
			/* long option */
			break;
		case 'd':
			/* device node name */
			device = strdup(optarg);
			break;
		case 'a':
			/* RAM address on the AXI bus in bytes */
			address = getopt_integer(optarg);
			break;
			/* RAM size in bytes */
		case 's':
			size = getopt_integer(optarg);
			break;
		case 'o':
			offset = getopt_integer(optarg) & 4095;
			break;
			/* count */
		case 'c':
			count = getopt_integer(optarg);
			break;
			/* count */
        case 'f':
            ofname = strdup(optarg);
            break;
        case 'u':
            uddofname = strdup(optarg);
            break;
			/* print usage help and exit */
    		case 'x':
			no_write++;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'h':
		default:
			usage(argv[0]);
			exit(0);
			break;
		}
	}
	if (verbose)
	fprintf(stdout,
		"dev %s, addr 0x%lx, size 0x%lx, offset 0x%lx, count %lu\n",
		device, address, size, offset, count);

	return test_dma(device, address, size, offset, count, ofname, uddofname);
}

static int test_dma(char *devname, uint64_t addr, uint64_t size,
		    uint64_t offset, uint64_t count, char *ofname, char *uddofname)
{
	ssize_t rc;
	uint64_t i;
	char *buffer = NULL;
	char *allocated = NULL;
	struct timespec ts_start, ts_end;
	int out_fd = -1;
	int out_udd_fd = -1;
	int fpga_fd = open(devname, O_RDWR | O_NONBLOCK);
	long total_time = 0;
	float result;
	float avg_time = 0;
	uint64_t num_pgs = 0;

	if (fpga_fd < 0) {
                fprintf(stderr, "unable to open device %s, %d.\n",
                        devname, fpga_fd);
		perror("open device");
                return -EINVAL;
        }

	/* create file to write data to */
	if (ofname) {
        out_fd = open(ofname, O_RDWR | O_CREAT | O_TRUNC | O_SYNC,
                0666);
        if (out_fd < 0) {
                        fprintf(stderr, "unable to open output file %s, %d.\n",
                                ofname, out_fd);
            perror("open output file");
                        rc = -EINVAL;
                        goto out;
                }
        out_udd_fd = open(uddofname, O_RDWR | O_CREAT | O_TRUNC | O_SYNC,
                0666);
        if (out_udd_fd < 0) {
                        fprintf(stderr, "unable to open output file %s, %d.\n",
                                uddofname, out_udd_fd);
            perror("open output file");
                        rc = -EINVAL;
                        goto out;
                }
	}

	num_pgs = (((size + offset) * count) >> 12) + 1;
	posix_memalign((void **)&allocated, 4096 /*alignment */ , num_pgs * 4096);
	if (!allocated) {
		fprintf(stderr, "OOM %lu.\n", num_pgs * 4096);
		rc = -ENOMEM;
		goto out;
	}

	if (verbose)
	    fprintf(stdout, "host buffer 0x%lx, %p.\n", size + 4096, buffer);

    buffer = allocated;
    rc = read_to_buffer(devname, fpga_fd, buffer, ((size + offset) * count), addr);
    if (rc < 0)
        goto out;
	for (i = 0; i < count; i++) {
		rc = clock_gettime(CLOCK_MONOTONIC, &ts_start);
		/* lseek & read data from AXI MM into buffer using SGDMA */
		clock_gettime(CLOCK_MONOTONIC, &ts_end);

		/* subtract the start time from the end time */
		timespec_sub(&ts_end, &ts_start);
		total_time += ts_end.tv_nsec;
		/* a bit less accurate but side-effects are accounted for */
		if (verbose)
		fprintf(stdout,
			"#%lu: CLOCK_MONOTONIC %ld.%09ld sec. read %ld bytes\n",
			i, ts_end.tv_sec, ts_end.tv_nsec, size);

		/* file argument given? */
		if ((out_udd_fd >= 0) & (no_write == 0)) {
		    rc = write_from_buffer(uddofname, out_udd_fd, buffer,
			     offset, i*offset);
		    if (rc < 0)
			goto out;
		}
		buffer += offset; /* point to actual data */
		/* file argument given? */
		if ((out_fd >= 0) & (no_write == 0)) {
			rc = write_from_buffer(ofname, out_fd, buffer,
					 size, i*size);
			if (rc < 0)
				goto out;
		}
		buffer += size; /* point to next packet */
	}
	avg_time = (float)total_time/(float)count;
	result = ((float)size)*1000/avg_time;
	if (verbose)
	printf("** Avg time device %s, total time %ld nsec, avg_time = %f, size = %lu, BW = %f \n",
		devname, total_time, avg_time, size, result);
	printf("** Average BW = %lu, %f\n", size, result);
	rc = 0;

out:
	close(fpga_fd);
    if (out_fd >= 0)
        close(out_fd);
    if (out_udd_fd >= 0)
        close(out_udd_fd);
	free(allocated);

	return rc;
}