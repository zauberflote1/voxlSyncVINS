
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "f32_ringbuf.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#ifndef unlikely
#define unlikely(x) __builtin_expect (!!(x), 0)
#endif

f32_ringbuf_t f32_ringbuf_empty(void)
{
    f32_ringbuf_t out = F32_RINGBUF_INITIALIZER;
    return out;
}


int f32_ringbuf_alloc(f32_ringbuf_t* buf, int size)
{
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in f32_ringbuf_alloc, received NULL pointer\n");
        return -1;
    }
    if(unlikely(size<2)){
        fprintf(stderr,"ERROR in f32_ringbuf_alloc, size must be >=2\n");
        return -1;
    }
    // if it's already allocated, nothing to do
    if(buf->initialized && buf->size==size && buf->d!=NULL) return 0;
    // make sure it's zero'd out
    buf->size = 0;
    buf->index = 0;
    buf->initialized = 0;
    // free memory and allocate fresh
    free(buf->d);
    buf->d = (float*)calloc(size,sizeof(float));
    if(buf->d==NULL){
        fprintf(stderr,"ERROR in f32_ringbuf_alloc, failed to allocate memory\n");
        return -1;
    }
    // write out other details
    buf->size = size;
    buf->initialized = 1;
    return 0;
}


int f32_ringbuf_free(f32_ringbuf_t* buf)
{
    f32_ringbuf_t new = F32_RINGBUF_INITIALIZER;
    if(unlikely(buf==NULL)){
        fprintf(stderr, "ERROR in f32_ringbuf_free, received NULL pointer\n");
        return -1;
    }
    if(buf->initialized) free(buf->d);
    *buf = new;
    return 0;
}


int f32_ringbuf_reset(f32_ringbuf_t* buf)
{
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr, "ERROR in f32_ringbuf_reset, received NULL pointer\n");
        return -1;
    }
    if(unlikely(!buf->initialized)){
        fprintf(stderr,"ERROR f32_ringbuf_reset, ringbuf uninitialized\n");
        return -1;
    }
    // wipe the data and index
    memset(buf->d,0,buf->size*sizeof(float));
    buf->index=0;
    return 0;
}


int f32_ringbuf_insert(f32_ringbuf_t* buf, float val)
{
    int new_index;
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in f32_ringbuf_insert, received NULL pointer\n");
        return -1;
    }
    if(unlikely(!buf->initialized)){
        fprintf(stderr,"ERROR in f32_ringbuf_insert, ringbuf uninitialized\n");
        return -1;
    }
    // increment index and check for loop-around
    new_index=buf->index+1;
    if(new_index>=buf->size) new_index=0;
    // write out new value
    buf->d[new_index]=val;
    buf->index=new_index;
    return 0;
}


float f32_ringbuf_get_value(f32_ringbuf_t* buf, int pos)
{
    int return_index;
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in f32_ringbuf_get_value, received NULL pointer\n");
        return -1.0f;
    }
    if(unlikely(pos<0 || pos>buf->size-1)){
        fprintf(stderr,"ERROR in f32_ringbuf_get_value, position out of bounds\n");
        return -1.0f;
    }
    if(unlikely(!buf->initialized)){
        fprintf(stderr,"ERROR in f32_ringbuf_get_value, ringbuf uninitialized\n");
        return -1.0f;
    }
    // check for looparound
    return_index=buf->index-pos;
    if(return_index<0) return_index+=buf->size;
    return buf->d[return_index];
}


int f32_ringbuf_copy_out_n_newest(f32_ringbuf_t* buf, int n, float* out)
{
    // sanity checks
    if(unlikely(buf==NULL)){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(n<1 || n>buf->size)){
        fprintf(stderr,"ERROR in %s, position out of bounds\n", __FUNCTION__);
        return -1;
    }
    if(unlikely(!buf->initialized)){
        fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
        return -1;
    }

    // find where to start copying from
    int start = buf->index - n + 1;
    if(start<0) start += buf->size;

    // copy the first chunk from oldest data
    int n_first_copy = buf->size-start;
    if(n_first_copy>n) n_first_copy = n;
    memcpy(out, &buf->d[start], n_first_copy*sizeof(float));

    // see if a second copy is needed due to wrap
    if(n_first_copy < n){
        int n_second_copy = n-n_first_copy;
        memcpy(&out[n_first_copy], buf->d, n_second_copy*sizeof(float));
    }

    return 0;
}
