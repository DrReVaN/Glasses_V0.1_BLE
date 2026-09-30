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
static uint32_t backup;
static bool backup_write_fails;
static unsigned jumps, cap_inits, cap_reads;
static uint8_t cap_samples[2];
static int cap_status;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_PWR_EnableBkUpAccess(void) {}
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t r, uint32_t v) {
    (void)h; assert(r == RTC_BKP_DR6); if (!backup_write_fails) backup = v;
}
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h, uint32_t r) {
    (void)h; assert(r == RTC_BKP_DR6); return backup;
}
int CAP1203_Init(I2C_HandleTypeDef *handle) {
    assert(handle == &hi2c1); ++cap_inits; return cap_status;
}
int CAP1203_ReadTouch(uint8_t *pads) {
    assert(cap_reads < 2); *pads = cap_samples[cap_reads++]; return cap_status;
}
void Glasses_HostJumpApplication(uint32_t sp, uint32_t entry) {
    assert(sp == 0x20007800 && entry == 0x08010141); ++jumps;
}
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
static void boot_route(uint32_t request, uint8_t first, uint8_t second,
                       unsigned expected_jumps, unsigned expected_reads) {
    backup = request; cap_samples[0] = first; cap_samples[1] = second;
    jumps = cap_inits = cap_reads = 0;
    Glasses_BootTryApplication();
    assert(jumps == expected_jumps && cap_reads == expected_reads);
}
static void test_boot_routes(void) {
    GlassesImage *m = (void *)GLASSES_META_ADDRESS;
    GlassesImage valid = *m;
    boot_route(0, 0, 0, 1, 1); /* ordinary start */
    boot_route(0, 2, 2, 0, 2); /* genuine held recovery pad */
    boot_route(0, 2, 0, 1, 2); /* latched released pad is not a recovery request */
    boot_route(GLASSES_BOOT_REQUEST, 0, 0, 0, 0);
    assert(backup == 0 && cap_inits == 0); /* OTA entry is consumed once */
    boot_route(backup, 0, 0, 1, 1);
    boot_route(GLASSES_BOOT_APPLICATION, 2, 2, 1, 0);
    assert(backup == 0 && cap_inits == 0); /* OTA commit skips touch, not validation */
    m->crc ^= 1;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    *m = valid; m->magic = 0;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    *m = valid; m->format = 2;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    *m = valid; m->size = GLASSES_APP_LIMIT - GLASSES_APP_ADDRESS + 1;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    *m = valid;
    uint32_t *vectors = (void *)GLASSES_APP_ADDRESS, saved_sp = vectors[0];
    vectors[0] = 0;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    vectors[0] = saved_sp;
    glasses_fault[0] = 0x53474631; glasses_fault[1] = 3;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    glasses_fault[0] = 0; glasses_reset_flags = RCC_CSR_IWDGRSTF;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    glasses_reset_flags = 0;
    backup_write_fails = true;
    boot_route(GLASSES_BOOT_APPLICATION, 0, 0, 0, 0);
    assert(backup == GLASSES_BOOT_APPLICATION);
    backup_write_fails = false; cap_status = HAL_ERROR;
    boot_route(0, 0, 0, 1, 0); /* a missing sensor cannot block a valid image */
    cap_status = HAL_OK;
    backup_write_fails = true; backup = 0; resets = 0;
    Glasses_OtaReboot(); assert(backup == 0 && resets == 0);
    backup_write_fails = false;
    Glasses_OtaReboot(); assert(backup == GLASSES_BOOT_REQUEST && resets == 1);
    resets = 0; backup = 0;
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
    glasses_fault[0] = 0x53474631; glasses_fault[1] = 3;
    Glasses_OtaWrite(3,1,3,(const uint8_t*)"END1",4);drain();assert(!error);
    GlassesImage *m=(void *)GLASSES_META_ADDRESS;
    assert(m->magic==GLASSES_IMAGE_MAGIC && m->size==sizeof(image) && m->crc==crc);
    assert(!memcmp((void *)GLASSES_APP_ADDRESS,image,sizeof(image)));
    assert(backup == GLASSES_BOOT_APPLICATION && glasses_fault[0] == 0);
    Glasses_OtaDisconnected(); /* app disconnect must not cancel the reboot */
    tick+=498;Glasses_OtaProcess();assert(resets==0);
    tick+=1;Glasses_OtaProcess();assert(resets==1);
    test_boot_routes();
    Glasses_OtaInit(); begin(image,sizeof(image),crc); upload(image,sizeof(image));
    backup_write_fails = true;
    Glasses_OtaWrite(3,1,3,(const uint8_t*)"END1",4);drain();assert(error);
    tick+=500;Glasses_OtaProcess();assert(resets==0);
    backup_write_fails = false;
    Glasses_OtaInit();begin(image,sizeof(image),crc);
    Glasses_OtaWrite(2,1,2,p,20);tick+=10001;Glasses_OtaProcess();assert(error);
    assert(((GlassesImage *)GLASSES_META_ADDRESS)->magic!=GLASSES_IMAGE_MAGIC);
    puts("OTA and actual boot-path tests passed: transfer errors, metadata-last commit, delayed reset after disconnect, explicit application start, held/released recovery pad, CRC/metadata/vector rejection, fault/watchdog recovery and backup-register failures.");
}
