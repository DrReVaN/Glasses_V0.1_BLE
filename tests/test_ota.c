#include "glasses_ota.h"
#include "glasses_core.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif
uint32_t glasses_fault[3], glasses_reset_flags;
RTC_HandleTypeDef hrtc;
I2C_HandleTypeDef hi2c1;
static uint32_t tick, erases, writes, resets;
static unsigned responses;
static int busy, fail;
static uint8_t error;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_PWR_EnableBkUpAccess(void) {}
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t r, uint32_t v) {(void)h;(void)r;(void)v;}
void NVIC_SystemReset(void) { ++resets; }
int aci_gatt_write_resp(uint16_t c,uint16_t a,uint8_t s,uint8_t e,uint8_t n,uint8_t *d) {
    (void)c;(void)a;(void)s;(void)n;(void)d; ++responses;error=e;return 0;
}
int Glasses_FlashErase(uint32_t address) {
    assert(address==GLASSES_META_ADDRESS || (address>=GLASSES_APP_ADDRESS && address<GLASSES_APP_LIMIT));
    if (busy) {--busy;return 1;} if(fail)return -1;
    memset((void *)(uintptr_t)address,255,4096);++erases;return 0;
}
int Glasses_FlashWrite(uint32_t address,uint64_t data) {
    assert((address&7)==0);
    assert((address>=GLASSES_META_ADDRESS && address<GLASSES_META_ADDRESS+16) ||
           (address>=GLASSES_APP_ADDRESS && address<GLASSES_APP_LIMIT));
    if(busy){--busy;return 1;}if(fail)return -1;
    uint8_t *p=(void *)(uintptr_t)address;
    for(unsigned i=0;i<8;++i){uint8_t b=((uint8_t *)&data)[i];assert((p[i]&b)==b);p[i]=b;}
    ++writes;return 0;
}
void Glasses_OtaWrite(uint8_t kind,uint16_t conn,uint16_t attr,const uint8_t *data,uint8_t len);
static void drain(void) {
    unsigned initial=responses;
    for(unsigned i=0;i<20000 && responses==initial;++i){Glasses_OtaProcess();++tick;}
    assert(responses==initial+1);
}
static void begin(const uint8_t *image,unsigned size,uint32_t crc) {
    uint8_t b[12]={'S','G','U','1'};memcpy(b+4,&size,4);memcpy(b+8,&crc,4);
    Glasses_OtaWrite(1,1,1,b,12);drain();assert(!error);(void)image;
}
static void upload(const uint8_t *image,unsigned size) {
    for(uint32_t offset=0;offset<size;offset+=16){
        uint8_t p[20]; unsigned n=size-offset;if(n>16)n=16;
        memcpy(p,&offset,4);memcpy(p+4,image+offset,n);
        Glasses_OtaWrite(2,1,2,p,n+4);drain();assert(!error);
    }
}
int main(void) {
#ifdef _WIN32
    void *memory=VirtualAlloc((void *)0x08000000,0x40000,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE);
#else
    void *memory=mmap((void *)0x08000000,0x40000,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED,-1,0);
#endif
    assert(memory==(void *)0x08000000);memset(memory,255,0x40000);
    uint8_t image[333];memset(image,0xA5,sizeof(image));
    uint32_t sp=0x20007800,entry=0x08010141;memcpy(image,&sp,4);memcpy(image+4,&entry,4);
    uint32_t crc=glasses_crc32(0xFFFFFFFFu,image,sizeof(image))^0xFFFFFFFFu;
    Glasses_OtaInit();
    uint8_t invalid[20]={0};Glasses_OtaWrite(1,1,1,invalid,12);assert(error && !erases && !writes);
    busy=5;begin(image,sizeof(image),crc);assert(erases==2);
    uint8_t p[20]={0};memcpy(p+4,image,16);Glasses_OtaWrite(2,1,2,p,20);drain();assert(!error);
    Glasses_OtaWrite(2,1,2,p,20);assert(error); /* duplicate offset */
    Glasses_OtaDisconnected();assert(((GlassesImage *)GLASSES_META_ADDRESS)->magic!=GLASSES_IMAGE_MAGIC);
    begin(image,sizeof(image),crc^1);upload(image,sizeof(image));
    Glasses_OtaWrite(3,1,3,(const uint8_t*)"END1",4);drain();assert(error);
    assert(((GlassesImage *)GLASSES_META_ADDRESS)->magic!=GLASSES_IMAGE_MAGIC);
    begin(image,sizeof(image),crc);
    fail=1;Glasses_OtaWrite(2,1,2,p,20);drain();assert(error);fail=0;
    begin(image,sizeof(image),crc);upload(image,sizeof(image));
    assert(((GlassesImage *)GLASSES_META_ADDRESS)->magic!=GLASSES_IMAGE_MAGIC);
    Glasses_OtaWrite(3,1,3,(const uint8_t*)"END1",4);drain();assert(!error);
    GlassesImage *m=(void *)GLASSES_META_ADDRESS;
    assert(m->magic==GLASSES_IMAGE_MAGIC && m->size==sizeof(image) && m->crc==crc);
    assert(!memcmp((void *)GLASSES_APP_ADDRESS,image,sizeof(image)));
    tick+=500;Glasses_OtaProcess();assert(resets==1);
    Glasses_OtaInit();begin(image,sizeof(image),crc);
    Glasses_OtaWrite(2,1,2,p,20);tick+=10001;Glasses_OtaProcess();assert(error);
    assert(((GlassesImage *)GLASSES_META_ADDRESS)->magic!=GLASSES_IMAGE_MAGIC);
    puts("OTA tests passed: busy flash, bounds, duplicate, disconnect, CRC rejection, failed write, final padding, metadata-last commit and timeout.");
}
