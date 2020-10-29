/**
 * Copyright(C) 2020, Isao Hara
 * All rights reserved
 *
 * License: Apache License v2.0
 */

#include <shm_image_save/shmem.h>

extern int errno;


/*
   Map shared memory
*/
void *map_shared_mem(int *shmid, int id, int len, int create)
{
  int shmid_;
  void *stat;

  if(create){
    shmid_ = shmget(id, len, IPC_CREAT|0666);
  }else{
    shmid_ =shmget(id, len, 0666);
  }

  *shmid = shmid_;
  if(shmid_ < 0){
    fprintf(stderr, "Error shared memory, id=%d, errno=%d\n", id, errno);
    return NULL;
  }

  stat=shmat(shmid_, NULL, 0666);
  if(stat < 0){
    fprintf(stderr, "Error: fail to map shared memory, id=%d, shmid=%d\n", id,shmid);
  }else{
    fprintf(stderr, "Success to map shared memory, id=%d, shmid=%d\n", id, shmid);
  }
  return stat;
}

/*
 Unmap shared memory
 */
int unmap_shared_mem(const void *addr)
{
  int res;
  if((res = shmdt(addr)) < 0){
    fprintf(stderr, "Error: fail to unmap shared memory addr=%d\n", addr);
  }
  return res;
}


int release_shared_mem(int shmid)
{
  int res;
  if((res = shmctl(shmid, IPC_RMID, 0)) < 0){
    fprintf(stderr, "Error: fail to release shared memory id=%d\n", shmid);
  }
  return res;
}
