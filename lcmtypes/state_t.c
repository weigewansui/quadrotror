// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "state_t.h"

static int __state_t_hash_computed;
static int64_t __state_t_hash;

int64_t __state_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __state_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__state_t_get_hash;
    (void) cp;

    int64_t hash = (int64_t)0x17dbcbe9aecc16d0LL
         + __int64_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int16_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int16_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __state_t_get_hash(void)
{
    if (!__state_t_hash_computed) {
        __state_t_hash = __state_t_hash_recursive(NULL);
        __state_t_hash_computed = 1;
    }

    return __state_t_hash;
}

int __state_t_encode_array(void *buf, int offset, int maxlen, const state_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, p[element].position, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, p[element].velocity, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, p[element].accel, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, p[element].angle, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, p[element].angular_vel, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, p[element].angular_accel, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, p[element].rpm, 4);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int state_t_encode(void *buf, int offset, int maxlen, const state_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __state_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __state_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __state_t_encoded_array_size(const state_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].timestamp), 1);

        size += __int32_t_encoded_array_size(p[element].position, 3);

        size += __int32_t_encoded_array_size(p[element].velocity, 3);

        size += __int16_t_encoded_array_size(p[element].accel, 3);

        size += __int32_t_encoded_array_size(p[element].angle, 3);

        size += __int32_t_encoded_array_size(p[element].angular_vel, 3);

        size += __int32_t_encoded_array_size(p[element].angular_accel, 3);

        size += __int16_t_encoded_array_size(p[element].rpm, 4);

    }
    return size;
}

int state_t_encoded_size(const state_t *p)
{
    return 8 + __state_t_encoded_array_size(p, 1);
}

int __state_t_decode_array(const void *buf, int offset, int maxlen, state_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, p[element].position, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, p[element].velocity, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, p[element].accel, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, p[element].angle, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, p[element].angular_vel, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, p[element].angular_accel, 3);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, p[element].rpm, 4);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __state_t_decode_array_cleanup(state_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].timestamp), 1);

        __int32_t_decode_array_cleanup(p[element].position, 3);

        __int32_t_decode_array_cleanup(p[element].velocity, 3);

        __int16_t_decode_array_cleanup(p[element].accel, 3);

        __int32_t_decode_array_cleanup(p[element].angle, 3);

        __int32_t_decode_array_cleanup(p[element].angular_vel, 3);

        __int32_t_decode_array_cleanup(p[element].angular_accel, 3);

        __int16_t_decode_array_cleanup(p[element].rpm, 4);

    }
    return 0;
}

int state_t_decode(const void *buf, int offset, int maxlen, state_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __state_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __state_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int state_t_decode_cleanup(state_t *p)
{
    return __state_t_decode_array_cleanup(p, 1);
}

int __state_t_clone_array(const state_t *p, state_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].timestamp), &(q[element].timestamp), 1);

        __int32_t_clone_array(p[element].position, q[element].position, 3);

        __int32_t_clone_array(p[element].velocity, q[element].velocity, 3);

        __int16_t_clone_array(p[element].accel, q[element].accel, 3);

        __int32_t_clone_array(p[element].angle, q[element].angle, 3);

        __int32_t_clone_array(p[element].angular_vel, q[element].angular_vel, 3);

        __int32_t_clone_array(p[element].angular_accel, q[element].angular_accel, 3);

        __int16_t_clone_array(p[element].rpm, q[element].rpm, 4);

    }
    return 0;
}

state_t *state_t_copy(const state_t *p)
{
    state_t *q = (state_t*) malloc(sizeof(state_t));
    __state_t_clone_array(p, q, 1);
    return q;
}

void state_t_destroy(state_t *p)
{
    __state_t_decode_array_cleanup(p, 1);
    free(p);
}

int state_t_publish(lcm_t *lc, const char *channel, const state_t *p)
{
      int max_data_size = state_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = state_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _state_t_subscription_t {
    state_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void state_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    state_t p;
    memset(&p, 0, sizeof(state_t));
    status = state_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding state_t!!!\n", status);
        return;
    }

    state_t_subscription_t *h = (state_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    state_t_decode_cleanup (&p);
}

state_t_subscription_t* state_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    state_t_handler_t f, void *userdata)
{
    state_t_subscription_t *n = (state_t_subscription_t*)
                       malloc(sizeof(state_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 state_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg state_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int state_t_subscription_set_queue_capacity (state_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int state_t_unsubscribe(lcm_t *lcm, state_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe state_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

