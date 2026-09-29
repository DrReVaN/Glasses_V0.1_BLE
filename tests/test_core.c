#include "glasses_core.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static void send(GlassesRx *r, const uint8_t *text, size_t size, uint32_t now) {
    size_t offset; uint8_t packet[20]; unsigned total = (unsigned)((size + 17) / 18);
    for (offset = 0; offset < size; offset += 18) {
        size_t len = size - offset; if (len > 18) len = 18;
        packet[0] = (uint8_t)(offset / 18); packet[1] = (uint8_t)total;
        memcpy(packet + 2, text + offset, len);
        assert(glasses_rx_push(r, packet, len + 2, now++));
    }
}
static void receiver(void) {
    GlassesRx rx = {0}; char out[GLASSES_TEXT_SIZE];
    uint8_t packet[20] = {0,2};
    memset(packet + 2, 'A', 18);
    assert(!glasses_rx_push(&rx, NULL, 0, 0));
    assert(!glasses_rx_push(&rx, packet, 2, 0));
    assert(glasses_rx_push(&rx, packet, 20, 0)); assert(rx.count == 0);
    packet[0] = 1;
    assert(glasses_rx_push(&rx, packet, 20, 1)); assert(rx.count == 1);
    assert(glasses_rx_pop(&rx, out)); assert(strlen(out) == 36); assert(!glasses_rx_pop(&rx, out));
    assert(!glasses_rx_push(&rx, packet, 20, 2)); /* fragment without start */
    packet[0] = 0;
    assert(glasses_rx_push(&rx, packet, 20, UINT32_MAX - 10));
    glasses_rx_expire(&rx, 3010); assert(!rx.total);
    send(&rx, (const uint8_t *)"Hallo", 5, 0); assert(glasses_rx_pop(&rx, out)); assert(!strcmp(out,"Hallo"));
    /* UTF-8 umlaut straddles a fragment boundary. */
    const uint8_t umlaut[] = "12345678901234567\xC3\xA4\xF0\x9F\x98\x80";
    send(&rx, umlaut, sizeof(umlaut)-1, 0); assert(glasses_rx_pop(&rx, out)); assert(!strcmp(out,"12345678901234567ae?"));
    uint8_t longtext[252]; memset(longtext,'X',sizeof(longtext));
    send(&rx, longtext, sizeof(longtext), 0); assert(glasses_rx_pop(&rx,out)); assert(strlen(out)==127); assert(rx.truncated==1);
    packet[0]=0;packet[1]=15; assert(!glasses_rx_push(&rx,packet,20,0));
    for (int i=0;i<6;++i) { uint8_t c=(uint8_t)('0'+i);send(&rx,&c,1,0); }
    assert(rx.count==4 && rx.dropped==2); assert(glasses_rx_pop(&rx,out)); assert(!strcmp(out,"2"));
    packet[0]=0;packet[1]=1;packet[2]=0xC0;packet[3]=0xAF;assert(!glasses_rx_push(&rx,packet,4,0));
    packet[2]='A';packet[3]=0;packet[4]='B';assert(!glasses_rx_push(&rx,packet,5,0));
    packet[4]=0;assert(glasses_rx_push(&rx,packet,5,0));
    /* Actual C parser exercised with random short, oversized and unordered frames. */
    struct { uint32_t before; GlassesRx rx; uint32_t after; } guarded = {0x12345678,{0},0x87654321};
    srand(123);
    for (unsigned i=0;i<100000;++i) {
        uint8_t fuzz[64]; size_t n=rand()%65;
        for(size_t j=0;j<n;++j) fuzz[j]=(uint8_t)rand();
        glasses_rx_push(&guarded.rx,fuzz,n,i*17);
        if(i%7==0) glasses_rx_pop(&guarded.rx,out);
        assert(guarded.before==0x12345678 && guarded.after==0x87654321);
        assert(guarded.rx.used<=GLASSES_RX_SIZE && guarded.rx.count<=GLASSES_QUEUE_SIZE);
    }
}
static void clock_tests(void) {
    GlassesClock c={0};
    assert(!glasses_clock_set(&c,(const uint8_t*)"24600101",8,0));
    assert(!glasses_clock_set(&c,(const uint8_t*)"12003104",8,0));
    assert(!glasses_clock_set(&c,(const uint8_t*)"120029022023",12,0));
    assert(glasses_clock_set(&c,(const uint8_t*)"235928022024",12,UINT32_MAX-500));
    glasses_clock_tick(&c,59499); assert(c.day==29 && c.hour==0 && c.minute==0);
    glasses_clock_tick(&c,86459499); assert(c.month==3 && c.day==1);
    assert(glasses_clock_set(&c,(const uint8_t*)"235931122026",12,0));
    glasses_clock_tick(&c,60000); assert(c.year==2027 && c.month==1 && c.day==1);
    assert(!glasses_clock_set(&c,(const uint8_t*)"abcd0101",8,0));
    assert(!glasses_clock_set(&c,(const uint8_t*)"120001",6,0));
}
static void touch_tests(void) {
    GlassesTouch t={0};
    assert(glasses_touch_poll(&t,2,0)==GLASSES_TOUCH_NONE);
    assert(glasses_touch_poll(&t,0,30)==GLASSES_TOUCH_NONE);
    assert(glasses_touch_poll(&t,2,40)==GLASSES_TOUCH_NONE);
    assert(glasses_touch_poll(&t,2,100)==GLASSES_TOUCH_NONE);
    assert(glasses_touch_poll(&t,2,2339)==GLASSES_TOUCH_NONE);
    assert(glasses_touch_poll(&t,2,2340)==GLASSES_POWER);
    assert(glasses_touch_poll(&t,2,9000)==GLASSES_TOUCH_NONE); /* OFF transition only once */
    glasses_touch_poll(&t,0,10000);glasses_touch_poll(&t,0,10060);
    glasses_touch_poll(&t,5,11000);glasses_touch_poll(&t,5,11060);
    assert(glasses_touch_poll(&t,5,12060)==GLASSES_TOUCH_NONE);
    assert(glasses_touch_poll(&t,5,14060)==GLASSES_PAIR);
    assert(glasses_touch_poll(&t,5,15060)==GLASSES_TOUCH_NONE);
}
int main(void) {
    receiver(); clock_tests(); touch_tests();
    assert((glasses_crc32(0xFFFFFFFFu,(const uint8_t*)"123456789",9)^0xFFFFFFFFu)==0xCBF43926);
    assert(glasses_image_vectors_valid(0x20007800,0x08010201,0x1000));
    assert(!glasses_image_vectors_valid(0xFFFFFFFF,0x08010201,0x1000));
    assert(!glasses_image_vectors_valid(0x20007800,0x08000201,0x1000));
    assert(!glasses_image_vectors_valid(0x20007800,0x08010200,0x1000));
    assert(!glasses_image_vectors_valid(0x20007800,0x08010201,0x30001));
    puts("Core tests passed (including 100000 malformed frame cases).");
}
