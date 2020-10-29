/**
 * Copyright(C) 2020, Isao Hara
 * All rights reserved
 *
 * License: Apache License v2.0
 */

#ifndef SHM_MEM_H_
#define SHM_MEM_H_
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/time.h>

#include <errno.h>

#define MAX_FOOD_DATA 10

#define SHM_IMAGE_ID (234567)
#define SHM_IMAGE_LEN (30000000)
#define MAX_MSGS (2)

#ifdef __cplusplus
extern "C" {
#endif
void *map_shared_mem(int *shmid, int id, int len, int create);
int unmap_shared_mem(const void *addr);
int release_shared_mem(int shmid);
#ifdef __cplusplus
}
#endif

#endif
