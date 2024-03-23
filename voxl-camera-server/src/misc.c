/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#define _GNU_SOURCE // for non-posix cpu affinity stuff

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>      // for O_WRONLY & O_RDONLY
#include <unistd.h>
#include <sys/stat.h> // for mkdir
#include <sched.h>
#include <pthread.h>
#include <time.h>

#include <misc.h>


// Given a file path, create all constituent directories if missing
void CreateParentDirs(const char *file_path)
{
  char *dir_path = (char *) malloc(strlen(file_path) + 1);
  const char *next_sep = strchr(file_path, '/');
  while (next_sep != NULL) {
    int dir_path_len = next_sep - file_path;
    memcpy(dir_path, file_path, dir_path_len);
    dir_path[dir_path_len] = '\0';
    mkdir(dir_path, S_IRWXU|S_IRWXG|S_IROTH);
    next_sep = strchr(next_sep + 1, '/');
  }
  free(dir_path);
}


int path_exists(char* path)
{
    // file exists
    if(access(path, F_OK ) != -1 ) return 1;
    // file doesn't exist
    return 0;
}



void lock_this_thread_to_big_cores(void)
{
  cpu_set_t cpuset;
  pthread_t thread;
  thread = pthread_self();

  /* Set affinity mask to include CPUs 7 only */
  CPU_ZERO(&cpuset);

#ifdef PLATFORM_APQ8096
  CPU_SET(2, &cpuset);
  CPU_SET(3, &cpuset);
#else
  CPU_SET(4, &cpuset);
  CPU_SET(5, &cpuset);
  CPU_SET(6, &cpuset);
  CPU_SET(7, &cpuset);
#endif


  if(pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)){
    perror("pthread_setaffinity_np");
  }

  // // Check the actual affinity mask assigned to the thread
  // if(pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset)){
  //   perror("pthread_getaffinity_np");
  // }
  // printf("thread %X is now locked to the following cores:", (unsigned int)thread);
  // for (int j = 0; j < CPU_SETSIZE; j++){
  //   if(CPU_ISSET(j, &cpuset)) printf(" %d", j);
  // }
  // printf("\n");

  return;
}





int64_t time_monotonic_ns()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_MONOTONIC, &ts)){
        fprintf(stderr,"ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}


int64_t start_clock(void)
{
    return time_monotonic_ns();
}


void stop_clock_and_print_time(char* name, int64_t t_start, int pixels)
{
    int64_t t_ns = time_monotonic_ns() - t_start;
    double nanoseconds_per_pixel = (double)(t_ns)/(double)(pixels);
    printf("%s took %0.2fms (%0.1f ns/pixel)\n", \
        name, (double)(t_ns)/1000000.0, nanoseconds_per_pixel);
}
