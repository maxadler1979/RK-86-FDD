#include "fdd_i2c.h"
#include "i2c_soft.h"

/* vinxru error codes (match fs.h / main) */
#define ERR_DISK_ERR        2
#define ERR_NO_PATH         4
#define ERR_NO_FREE_SPACE   6
#define ERR_FILE_EXISTS     8
#define ERR_INVALID_COMMAND 12

static uint8_t rxbuf[FDD_RX_MAX];

/* Busy-wait ~1 ms @ 16 MHz — works with IRQs off (handleSession). */
static void fdd_delay_ms(uint16_t ms)
{
    while (ms--) {
        uint16_t n = 1600;
        while (n--)
            ;
    }
}

uint8_t fdd_map_error(uint8_t st)
{
    switch (st) {
    case FDD_ST_OK:
    case FDD_ST_EOF:
        return 0;
    case FDD_ST_NOTFOUND:
        return ERR_NO_PATH;
    case FDD_ST_NOTREADY:
    case FDD_ST_ERROR:
        return ERR_DISK_ERR;
    case 0x1E:
        return 0x1E;
    case FDD_ST_READONLY:
    case FDD_ST_DENIED:
        return ERR_NO_FREE_SPACE;
    case FDD_ST_EXISTS:
        return ERR_FILE_EXISTS;
    case FDD_ST_INVALID:
    default:
        return ERR_INVALID_COMMAND;
    }
}

void fdd_init(void)
{
    i2c_soft_init();
}

/* Write cmd, wait until slave not BUSY, read response into rxbuf.
 * short_timeout: use for GetStatus (no Arduino → fail fast). */
static uint8_t fdd_xfer_ex(const uint8_t *cmd, uint8_t cmd_len, uint8_t want_len,
                           uint16_t max_tries)
{
    uint16_t tries;
    uint8_t n;

    if (i2c_soft_write(FDD_I2C_ADDR, cmd, cmd_len))
        return FDD_ST_ERROR;

    (void)want_len;
    for (tries = 0; tries < max_tries; tries++) {
        fdd_delay_ms(1);
        /* Slave always answers 32 bytes: short reads wedge AVR TWI. */
        n = FDD_RX_MAX;
        if (i2c_soft_read(FDD_I2C_ADDR, rxbuf, n))
            continue;
        if (rxbuf[0] == FDD_ST_BUSY)
            continue;
        return rxbuf[0];
    }
    return 0x1E; /* no answer — distinct from FatFs / disk errors */
}

static uint8_t fdd_xfer(const uint8_t *cmd, uint8_t cmd_len, uint8_t want_len)
{
    /* File/motor ops may spin the drive for seconds */
    return fdd_xfer_ex(cmd, cmd_len, want_len, 20000);
}

static uint8_t fdd_cmd0(uint8_t c)
{
    uint8_t cmd[1];
    cmd[0] = c;
    return fdd_xfer(cmd, 1, 1);
}

static uint8_t fdd_cmd_path(uint8_t c, const char *path)
{
    uint8_t cmd[32];
    uint8_t i = 0;

    cmd[0] = c;
    if (path) {
        while (path[i] && i < 30) {
            cmd[1 + i] = (uint8_t)path[i];
            i++;
        }
    }
    cmd[1 + i] = 0;
    return fdd_xfer(cmd, (uint8_t)(2 + i), 1);
}

uint8_t fdd_get_status(FddStatus *st)
{
    uint8_t cmd = FDD_CMD_GET_STATUS;
    uint8_t s;

    /* ~200 ms — Arduino must answer fast; haveDisk no longer blocks GetStatus */
    s = fdd_xfer_ex(&cmd, 1, 6, 200);
    if (s != FDD_ST_OK)
        return s;
    if (st) {
        st->drive_type = rxbuf[1];
        st->have_disk = rxbuf[2];
        st->write_protected = rxbuf[3];
        st->motor = rxbuf[4];
        st->selected_drive = rxbuf[5];
    }
    return FDD_ST_OK;
}

uint8_t fdd_motor_on(void)
{
    return fdd_cmd0(FDD_CMD_MOTOR_ON);
}

uint8_t fdd_motor_off(void)
{
    return fdd_cmd0(FDD_CMD_MOTOR_OFF);
}

uint8_t fdd_dir_start(const char *path)
{
    return fdd_cmd_path(FDD_CMD_DIR_START, path);
}

uint8_t fdd_dir_next(uint8_t *entry19)
{
    uint8_t cmd = FDD_CMD_DIR_NEXT;
    uint8_t s;
    uint8_t i;

    s = fdd_xfer(&cmd, 1, 19);
    if (entry19) {
        if (s == FDD_ST_OK) {
            for (i = 0; i < 19; i++)
                entry19[i] = rxbuf[i];
        } else {
            entry19[0] = s;
        }
    }
    return s;
}

uint8_t fdd_open_read(const char *path)
{
    return fdd_cmd_path(FDD_CMD_OPEN_READ, path);
}

uint8_t fdd_open_write(const char *path)
{
    return fdd_cmd_path(FDD_CMD_OPEN_WRITE, path);
}

uint8_t fdd_close(void)
{
    return fdd_cmd0(FDD_CMD_FILE_CLOSE);
}

uint8_t fdd_delete(const char *path)
{
    return fdd_cmd_path(FDD_CMD_FILE_DELETE, path);
}

uint8_t fdd_mkdir(const char *path)
{
    return fdd_cmd_path(FDD_CMD_MKDIR, path);
}

uint8_t fdd_seek(uint32_t offset)
{
    uint8_t cmd[5];

    cmd[0] = FDD_CMD_FILE_SEEK;
    cmd[1] = (uint8_t)(offset);
    cmd[2] = (uint8_t)(offset >> 8);
    cmd[3] = (uint8_t)(offset >> 16);
    cmd[4] = (uint8_t)(offset >> 24);
    return fdd_xfer(cmd, 5, 1);
}

uint8_t fdd_get_size(uint32_t *size)
{
    uint8_t cmd = FDD_CMD_FILE_SIZE;
    uint8_t s;

    s = fdd_xfer(&cmd, 1, 5);
    if (s == FDD_ST_OK && size) {
        *size = (uint32_t)rxbuf[1]
              | ((uint32_t)rxbuf[2] << 8)
              | ((uint32_t)rxbuf[3] << 16)
              | ((uint32_t)rxbuf[4] << 24);
    }
    return s;
}

uint8_t fdd_tell(uint32_t *pos)
{
    uint8_t cmd = FDD_CMD_FILE_TELL;
    uint8_t s;

    s = fdd_xfer(&cmd, 1, 5);
    if (s == FDD_ST_OK && pos) {
        *pos = (uint32_t)rxbuf[1]
             | ((uint32_t)rxbuf[2] << 8)
             | ((uint32_t)rxbuf[3] << 16)
             | ((uint32_t)rxbuf[4] << 24);
    }
    return s;
}

uint8_t fdd_read(uint8_t *dst, uint8_t max_len, uint8_t *out_len)
{
    uint8_t cmd[2];
    uint8_t s;
    uint8_t n;
    uint8_t i;

    if (max_len > 30)
        max_len = 30;
    cmd[0] = FDD_CMD_FILE_READ;
    cmd[1] = max_len;
    s = fdd_xfer(cmd, 2, (uint8_t)(2 + max_len));
    if (s != FDD_ST_OK && s != FDD_ST_EOF)
        return s;
    n = rxbuf[1];
    if (n > max_len)
        n = max_len;
    if (dst) {
        for (i = 0; i < n; i++)
            dst[i] = rxbuf[2 + i];
    }
    if (out_len)
        *out_len = n;
    return s;
}

uint8_t fdd_write(const uint8_t *src, uint8_t len)
{
    uint8_t cmd[32];
    uint8_t i;

    if (len > 30)
        len = 30;
    cmd[0] = FDD_CMD_FILE_WRITE;
    cmd[1] = len;
    for (i = 0; i < len; i++)
        cmd[2 + i] = src[i];
    return fdd_xfer(cmd, (uint8_t)(2 + len), 1);
}
