/*
 * FDD controller for Radio-86RK / Apogee BK01
 * palmira original software
 * STM8S103K3 — VV55 protocol (vinxru) + soft-I2C to Arduino FDC.
 *
 * ROM emu → CMD0 embedded sdbios → CMD1 ver → CMD2+ via floppy FatFs on Arduino.
 */

#include "stm8s.h"
#include "rk_images.h"
#include "i2c_soft.h"
#include "fdd_i2c.h"

#define ERR_START       0x40
#define ERR_WAIT        0x41
#define ERR_OK_DISK     0x42
#define ERR_OK_CMD      0x43
#define ERR_OK_READ     0x44
#define ERR_OK_ENTRY    0x45
#define ERR_OK_WRITE    0x46
#define ERR_OK_RKS      0x47
#define ERR_READ_BLOCK  0x4F

#define ERR_DISK_ERR        0x02
#define ERR_NOT_OPENED      0x03
#define ERR_NO_PATH         0x04
#define ERR_NO_DATA         0x09
#define ERR_MAX_FILES       0x0A
#define ERR_RECV_STRING     0x0B
#define ERR_INVALID_CMD     0x0C

#define O_OPEN   0
#define O_CREATE 1
#define O_MKDIR  2
#define O_DELETE 100
#define O_SWAP   101

#define DATA_CR1_MIX  ((uint8_t)0xCF) /* PP except PB4/PB5 true-OD */
/* Re-assert DDR every time: OD PB4/PB5 must stay outputs or pull-ups → |0x30 */
#define DATA_OUT()  do { \
        GPIOB->CR1 = DATA_CR1_MIX; \
        GPIOB->CR2 = 0x00; \
        GPIOB->DDR = 0xFF; \
    } while (0)
#define DATA_IN()   do { \
        GPIOB->DDR = 0x00; \
        GPIOB->CR1 = 0x00; \
        GPIOB->CR2 = 0x00; \
    } while (0)

static void dataPut(uint8_t c)
{
    DATA_OUT();
    GPIOB->ODR = c;
    /* True-OD: ODR=0 drives low; ODR=1 = Hi-Z (external pull-up).
     * Ensure DDR still 1 after any ISR/glitch. */
    GPIOB->DDR = 0xFF;
}

#define PATH_MAX  48
#define CHUNK_MAX 30

volatile uint32_t TimingDelay;
static uint8_t lastError;
static uint8_t protoAbort;
static char    pathBuf[PATH_MAX];
static uint8_t chunk[CHUNK_MAX];
static uint16_t readLength;
static uint16_t writeTotal;
static uint8_t  fileOpen; /* 1=read, 2=write */

uint8_t rom_ram[128];
uint8_t rom_addr_hi;

void romEmulation(void);

static uint8_t readAddr(void)
{
    uint8_t pd = GPIOD->IDR;
    uint8_t addr = (uint8_t)(GPIOC->IDR & 0xF0);

    addr |= (uint8_t)(pd & 0x01);
    addr |= (uint8_t)((pd & 0x04) >> 1);
    addr |= (uint8_t)((pd & 0x08) >> 1);
    addr |= (uint8_t)((pd & 0x10) >> 1);
    return addr;
}

static void romRamInit(void)
{
    uint8_t i;
    for (i = 0; i < 128; i++)
        rom_ram[i] = (i < boot_rk_img_len) ? boot_rk_img[i] : 0xFF;
}

/* Same edge as the ATmega controller: strobe 0→1→0, then the byte. */
static uint8_t waitClk(void)
{
    while ((GPIOC->IDR & 0x20) == 0)
        ;
    while ((GPIOC->IDR & 0x20) != 0)
        ;
    if ((readAddr() & 0x3F) != 0) {
        protoAbort = 1;
        return 0;
    }
    return 1;
}

static uint8_t sendByte(uint8_t c)
{
    if (!waitClk()) return 0;
    GPIOB->ODR = c;
    return 1;
}

static uint8_t sendStart(uint8_t c)
{
    if (!waitClk()) return 0;
    DATA_OUT();
    GPIOB->ODR = c;
    return 1;
}

static uint8_t recvStart(void)
{
    if (!waitClk()) return 0;
    DATA_IN();
    return 1;
}

static uint8_t recvByte(uint8_t *out)
{
    if (!waitClk()) return 0;
    *out = GPIOB->IDR;
    return 1;
}

static uint8_t sendBin(const uint8_t *p, uint16_t len)
{
    while (len--) {
        if (!sendByte(*p++)) return 0;
    }
    return 1;
}

static uint8_t recvBin(uint8_t *p, uint16_t len)
{
    while (len--) {
        if (!recvByte(p++)) return 0;
    }
    return 1;
}

static void recvString(void)
{
    uint8_t c;
    uint8_t i = 0;

    do {
        if (!recvByte(&c)) return;
        if (i < (PATH_MAX - 1))
            pathBuf[i++] = (char)c;
        else
            lastError = ERR_RECV_STRING;
    } while (c);
    if (i == 0 || pathBuf[i - 1] != 0)
        pathBuf[i] = 0;
}

/* ---- embedded sdbios (CMD 0) ---- */

static void sendRkFile(const uint8_t *img, uint16_t totalLen)
{
    uint8_t  a0, a1;
    uint16_t loadAddr, endAddr, payloadLen, off, chunkn;

    if (totalLen < 4) {
        lastError = ERR_DISK_ERR;
        return;
    }

    a0 = img[1];
    a1 = img[0];
    loadAddr = (uint16_t)a0 | ((uint16_t)a1 << 8);
    endAddr  = (uint16_t)img[3] | ((uint16_t)img[2] << 8);
    payloadLen = (uint16_t)(endAddr - loadAddr + 1);
    (void)loadAddr;

    if ((uint16_t)(4 + payloadLen) > totalLen)
        payloadLen = (uint16_t)(totalLen - 4);

    if (!sendByte(ERR_OK_RKS)) return;
    if (!sendByte(a0)) return;
    if (!sendByte(a1)) return;
    if (!sendByte(ERR_WAIT)) return;

    off = 4;
    while (payloadLen) {
        chunkn = payloadLen;
        if (chunkn > 256)
            chunkn = 256;
        if (!sendByte(ERR_READ_BLOCK)) return;
        if (!sendByte((uint8_t)(chunkn & 0xFF))) return;
        if (!sendByte((uint8_t)(chunkn >> 8))) return;
        if (!sendBin(&img[off], chunkn)) return;
        if (!sendByte(ERR_WAIT)) return;
        off = (uint16_t)(off + chunkn);
        payloadLen = (uint16_t)(payloadLen - chunkn);
    }
    if (!protoAbort)
        lastError = ERR_OK_READ;
}

static void cmdBoot(void)
{
    if (!sendStart(ERR_WAIT)) return;
    sendRkFile(sdbios_rk_img, sdbios_rk_img_len);
}

static char ver_text[16] = "I2C- STM8S103K3";

static void cmdVer(void)
{
    if (!sendStart(1)) return;
    if (!sendBin((const uint8_t *)ver_text, 16)) return;
    lastError = 0;
}

/* ---- stream file from Arduino as RKS or raw ---- */

static void readInt(uint8_t rks)
{
    uint8_t st, n, i;
    uint8_t hdr[4];
    uint8_t hdr_left = rks ? 4 : 0;
    uint16_t lengthFromFile = 0xFFFF;
    uint16_t blockLen;
    uint8_t *wptr;
    uint8_t tmp;

    while (readLength && !protoAbort && !lastError) {
        st = fdd_read(chunk, CHUNK_MAX, &n);
        if (st != FDD_ST_OK && st != FDD_ST_EOF) {
            if ((st & 0xF0) == 0x10)
                lastError = st;
            else
                lastError = fdd_map_error(st);
            return;
        }
        if (n == 0) {
            if (st == FDD_ST_EOF)
                break;
            lastError = ERR_DISK_ERR;
            return;
        }
        if (n > readLength)
            n = (uint8_t)readLength;
        readLength = (uint16_t)(readLength - n);

        wptr = chunk;
        blockLen = n;

        if (hdr_left) {
            for (i = 0; i < n && hdr_left; i++) {
                hdr[4 - hdr_left] = chunk[i];
                hdr_left--;
            }
            if (hdr_left)
                continue; /* need more header bytes */

            /* full 4-byte RK header — Apogey endian swap */
            tmp = hdr[0]; hdr[0] = hdr[1]; hdr[1] = tmp;
            tmp = hdr[2]; hdr[2] = hdr[3]; hdr[3] = tmp;

            if (!sendByte(ERR_OK_RKS)) return;
            if (!sendBin(hdr, 2)) return;
            if (!sendByte(ERR_WAIT)) return;

            lengthFromFile = (uint16_t)(
                ((uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8))
                - ((uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8))
                + 1);

            wptr = &chunk[i];
            blockLen = (uint16_t)(n - i);
            if (blockLen > lengthFromFile)
                blockLen = lengthFromFile;
            lengthFromFile = (uint16_t)(lengthFromFile - blockLen);
            rks = 0;
        } else if (rks == 0 && lengthFromFile != 0xFFFF) {
            if (blockLen > lengthFromFile)
                blockLen = lengthFromFile;
            lengthFromFile = (uint16_t)(lengthFromFile - blockLen);
        }

        if (blockLen) {
            if (!sendByte(ERR_READ_BLOCK)) return;
            if (!sendByte((uint8_t)(blockLen & 0xFF))) return;
            if (!sendByte((uint8_t)(blockLen >> 8))) return;
            if (!sendBin(wptr, blockLen)) return;
            if (!sendByte(ERR_WAIT)) return;
        }

        if (st == FDD_ST_EOF)
            break;
        if (lengthFromFile == 0 && !hdr_left)
            break;
    }

    (void)fdd_close();
    fileOpen = 0;
    if (!protoAbort && !lastError)
        lastError = ERR_OK_READ;
}

static void cmdBootExec(uint8_t rks)
{
    uint8_t st;
    uint32_t sz;

    if (pathBuf[0] == 0) {
        /* default path for CMD0-from-disk unused; CMD0 uses embedded */
        pathBuf[0] = 'B'; pathBuf[1] = 'O'; pathBuf[2] = 'O'; pathBuf[3] = 'T';
        pathBuf[4] = '/'; pathBuf[5] = 'S'; pathBuf[6] = 'H'; pathBuf[7] = 'E';
        pathBuf[8] = 'L'; pathBuf[9] = 'L'; pathBuf[10] = '.'; pathBuf[11] = 'R';
        pathBuf[12] = 'K'; pathBuf[13] = 0;
    }

    st = fdd_open_read(pathBuf);
    if (st != FDD_ST_OK) {
        /* 0x1N is FatFs code in the low nibble (11=disk, 1D=not FAT, …). */
        if ((st & 0xF0) == 0x10)
            lastError = st;
        else
            lastError = fdd_map_error(st);
        return;
    }
    fileOpen = 1;

    st = fdd_get_size(&sz);
    if (st != FDD_ST_OK) {
        lastError = fdd_map_error(st);
        fdd_close();
        fileOpen = 0;
        return;
    }

    readLength = (sz > 0xFFFFUL) ? 0xFFFFU : (uint16_t)sz;
    if (readLength < 4) {
        lastError = ERR_DISK_ERR;
        fdd_close();
        fileOpen = 0;
        return;
    }

    readInt(rks);
}

static void cmdExec(void)
{
    recvString();
    if (!sendStart(ERR_WAIT)) return;
    if (lastError) return;
    cmdBootExec(1);
}

/* Convert "NAME.EXT" → 11-char FAT (8+3 spaces) into dst[11] */
static void toFat11(const uint8_t *name, uint8_t *dst)
{
    uint8_t i, j;

    for (i = 0; i < 11; i++)
        dst[i] = ' ';
    i = 0;
    j = 0;
    while (name[i] && name[i] != '.' && j < 8) {
        dst[j++] = name[i++];
    }
    if (name[i] == '.') {
        i++;
        j = 8;
        while (name[i] && j < 11)
            dst[j++] = name[i++];
    }
}

static void cmdFind(void)
{
    uint16_t n, left;
    uint8_t st;
    uint8_t entry[19];
    uint8_t info[20]; /* FILINFO2: fname[11], attr, size[4], time[4] */
    uint8_t attempt;
    const char *path;

    recvString();
    if (!recvBin((uint8_t *)&n, 2)) return;
    if (!sendStart(ERR_WAIT)) return;
    if (lastError) return;

    /* Root "/" is the same folder as an empty path. */
    path = pathBuf;
    if (path[0] == '/' && path[1] == 0)
        path = "";

    /*
     * The first listing after SHELL.RK often comes back empty: the disk
     * is still settling, or opening SHELL.IN failed. The shell does not
     * retry when the path is empty, so the panel stays blank until Enter.
     */
    for (attempt = 0; attempt < 2; attempt++) {
        if (path[0] != ':') {
            st = fdd_dir_start(path[0] ? path : (const char *)"");
            if (st != FDD_ST_OK) {
                if (attempt == 0)
                    continue;
                lastError = fdd_map_error(st);
                return;
            }
        }

        for (left = n; left; --left) {
            st = fdd_dir_next(entry);
            if (st == FDD_ST_EOF) {
                if (attempt == 0 && left == n && path[0] != ':')
                    break;
                lastError = ERR_OK_CMD;
                return;
            }
            if (st != FDD_ST_OK) {
                if (attempt == 0)
                    break;
                lastError = fdd_map_error(st);
                return;
            }

            toFat11(&entry[1], info);
            info[11] = entry[14]; /* attrib */
            info[12] = entry[15];
            info[13] = entry[16];
            info[14] = entry[17];
            info[15] = entry[18];
            info[16] = 0;
            info[17] = 0;
            info[18] = 0;
            info[19] = 0;

            if (!sendByte(ERR_OK_ENTRY)) return;
            if (!sendBin(info, 20)) return;
            if (!sendByte(ERR_WAIT)) return;
            attempt = 2; /* an entry was sent; do not restart */
        }
        if (attempt >= 2)
            break;
    }
    if (!lastError)
        lastError = ERR_OK_CMD;
}

static void cmdOpen(void)
{
    uint8_t mode;
    uint8_t st;

    if (!recvByte(&mode)) return;
    recvString();
    if (!sendStart(ERR_WAIT)) return;

    if (mode == O_SWAP) {
        lastError = ERR_INVALID_CMD;
    } else if (mode == O_DELETE) {
        st = fdd_delete(pathBuf);
        lastError = fdd_map_error(st);
    } else if (mode == O_OPEN) {
        st = fdd_open_read(pathBuf);
        if (st == FDD_ST_OK)
            fileOpen = 1;
        lastError = fdd_map_error(st);
    } else if (mode == O_CREATE) {
        st = fdd_open_write(pathBuf);
        if (st == FDD_ST_OK)
            fileOpen = 2;
        lastError = fdd_map_error(st);
    } else if (mode == O_MKDIR) {
        st = fdd_mkdir(pathBuf);
        lastError = fdd_map_error(st);
    } else {
        lastError = ERR_INVALID_CMD;
    }

    if (!lastError)
        lastError = ERR_OK_CMD;
}

static void cmdLseek(void)
{
    uint8_t mode;
    uint32_t off;
    uint8_t st;
    uint32_t res = 0;

    if (!recvByte(&mode)) return;
    if (!recvBin((uint8_t *)&off, 4)) return;
    if (!sendStart(ERR_WAIT)) return;

    if (!fileOpen && mode < 100) {
        lastError = ERR_NOT_OPENED;
        return;
    }

    if (mode == 100) {
        st = fdd_get_size(&res);
    } else if (mode == 101 || mode == 102) {
        res = 0;
        st = FDD_ST_OK;
    } else if (mode == 0) {
        /* SEEK_SET */
        st = fdd_seek(off);
        if (st == FDD_ST_OK)
            st = fdd_tell(&res);
    } else if (mode == 1) {
        /* SEEK_CUR */
        st = fdd_tell(&res);
        if (st == FDD_ST_OK) {
            res += off;
            st = fdd_seek(res);
            if (st == FDD_ST_OK)
                st = fdd_tell(&res);
        }
    } else if (mode == 2) {
        /* SEEK_END */
        st = fdd_get_size(&res);
        if (st == FDD_ST_OK) {
            res += off;
            st = fdd_seek(res);
            if (st == FDD_ST_OK)
                st = fdd_tell(&res);
        }
    } else {
        lastError = ERR_INVALID_CMD;
        return;
    }

    if (st != FDD_ST_OK) {
        lastError = fdd_map_error(st);
        return;
    }

    if (!sendByte(ERR_OK_CMD)) return;
    if (!sendBin((uint8_t *)&res, 4)) return;
    lastError = 0;
}

static void cmdRead(void)
{
    uint32_t sz, pos, left;
    uint8_t st;

    if (!recvBin((uint8_t *)&readLength, 2)) return;
    if (!sendStart(ERR_WAIT)) return;

    if (!fileOpen) {
        lastError = ERR_NOT_OPENED;
        return;
    }

    st = fdd_get_size(&sz);
    if (st != FDD_ST_OK) {
        lastError = fdd_map_error(st);
        return;
    }
    st = fdd_tell(&pos);
    if (st != FDD_ST_OK) {
        lastError = fdd_map_error(st);
        return;
    }
    left = (sz > pos) ? (sz - pos) : 0;
    if (readLength > left)
        readLength = (uint16_t)left;

    readInt(0);
}

static void cmdWrite(void)
{
    uint8_t st;
    uint16_t got;
    uint8_t n;

    if (!recvBin((uint8_t *)&writeTotal, 2)) return;
    if (!sendStart(ERR_WAIT)) return;

    if (!fileOpen) {
        lastError = ERR_NOT_OPENED;
        return;
    }

    if (writeTotal == 0) {
        st = fdd_close();
        fileOpen = 0;
        lastError = fdd_map_error(st);
        if (!lastError)
            lastError = ERR_OK_CMD;
        return;
    }

    while (writeTotal && !protoAbort && !lastError) {
        got = writeTotal;
        if (got > CHUNK_MAX)
            got = CHUNK_MAX;

        if (!sendByte(ERR_OK_WRITE)) return;
        if (!sendByte((uint8_t)(got & 0xFF))) return;
        if (!sendByte((uint8_t)(got >> 8))) return;
        if (!recvStart()) return;
        if (!recvBin(chunk, got)) return;
        if (!sendStart(ERR_WAIT)) return;

        st = fdd_write(chunk, (uint8_t)got);
        if (st != FDD_ST_OK) {
            lastError = fdd_map_error(st);
            return;
        }
        writeTotal = (uint16_t)(writeTotal - got);
        (void)n;
    }

    if (!lastError)
        lastError = ERR_OK_CMD;
}

static void cmdMove(void)
{
    recvString();
    if (!sendStart(ERR_WAIT)) return;
    /* rename not on Arduino yet */
    lastError = ERR_INVALID_CMD;
    (void)sendStart(ERR_OK_WRITE);
}

static void gpioInit(void)
{
    GPIO_Init(GPIOD,
              (GPIO_Pin_TypeDef)(GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4),
              GPIO_MODE_IN_FL_NO_IT);
    GPIO_Init(GPIOC,
              (GPIO_Pin_TypeDef)(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7),
              GPIO_MODE_IN_FL_NO_IT);

    GPIO_Init(GPIOB,
              (GPIO_Pin_TypeDef)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                                 GPIO_PIN_6 | GPIO_PIN_7),
              GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOB, (GPIO_Pin_TypeDef)(GPIO_PIN_4 | GPIO_PIN_5),
              GPIO_MODE_OUT_OD_HIZ_FAST);
    DATA_OUT();
    dataPut(0x00);
}

static void handleSession(void)
{
    uint8_t cmd;

    protoAbort = 0;
    lastError = 0;

    DATA_OUT();
    GPIOB->ODR = ERR_START;

    if (!sendStart(ERR_START))
        goto done;
    if (!sendByte(ERR_WAIT))
        goto done;
    if (!sendByte(ERR_OK_DISK))
        goto done;

    if (!recvStart())
        goto done;
    if (!recvByte(&cmd))
        goto done;

    lastError = 0;
    switch (cmd) {
    case 0:
        cmdBoot();
        break;
    case 1:
        cmdVer();
        break;
    case 2:
        /* sdbios: load BOOT/SHELL.RK. 04 = not found, 02 = read error. */
        cmdExec();
        break;
    case 3:
        cmdFind();
        break;
    case 4:
        cmdOpen();
        break;
    case 5:
        cmdLseek();
        break;
    case 6:
        cmdRead();
        break;
    case 7:
        cmdWrite();
        break;
    case 8:
        cmdMove();
        break;
    default:
        lastError = ERR_DISK_ERR;
        break;
    }

    if (!protoAbort && lastError)
        (void)sendStart(lastError);

done:
    /* 8080 читает шину уже после спада CLK. Если обнулить её раньше,
     * вместо 44h он видит 00h и пишет «ошибка SD B1» (00h-4Fh).
     * volatile — иначе IAR выбрасывает пустой цикл и ошибка плавает. */
    {
        volatile uint16_t d = 8000;
        while (d--)
            ;
    }
    DATA_OUT();
    dataPut(0x00);
}

void delay_ms(uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0)
        ;
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0)
        TimingDelay--;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

int main(void)
{
    CLK_DeInit();
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI,
                          DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);

    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_PRESCALER_64, 0xFA);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    TIM4_Cmd(ENABLE);
    enableInterrupts();

    gpioInit();
    i2c_soft_init();
    {
        FddStatus st;
        /* One GetStatus before serving ROM. NACK returns at once. */
        if (fdd_get_status(&st) == FDD_ST_OK)
            ver_text[3] = '1';
        else
            ver_text[3] = '0';
    }
    romRamInit();
    DATA_OUT();
    dataPut(rom_ram[0]);

    {
        uint16_t ms;
        for (ms = 0; ms < 2000; ms++) {
            disableInterrupts();
            dataPut(rom_ram[readAddr() & 0x7F]);
            enableInterrupts();
            delay_ms(1);
        }
        /* Arduino setup() spins the drive before Wire.begin. Re-probe now. */
        {
            FddStatus st;
            if (fdd_get_status(&st) == FDD_ST_OK)
                ver_text[3] = '1';
        }
    }

    for (;;) {
        disableInterrupts();
        DATA_OUT();
        romEmulation();
        handleSession();
        enableInterrupts();
    }
}
