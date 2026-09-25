/*
 * Arduino FDC I2C client (slave 0x42) — FatFs on floppy, no local FAT.
 */
#ifndef FDD_I2C_H
#define FDD_I2C_H

#include "stm8s.h"

#define FDD_I2C_ADDR        0x42

#define FDD_CMD_GET_STATUS  0x02
#define FDD_CMD_DIR_START   0x10
#define FDD_CMD_DIR_NEXT    0x11
#define FDD_CMD_OPEN_READ   0x20
#define FDD_CMD_FILE_READ   0x21
#define FDD_CMD_OPEN_WRITE  0x22
#define FDD_CMD_FILE_WRITE  0x23
#define FDD_CMD_FILE_CLOSE  0x24
#define FDD_CMD_FILE_DELETE 0x25
#define FDD_CMD_FILE_SEEK   0x26
#define FDD_CMD_FILE_SIZE   0x27
#define FDD_CMD_FILE_TELL   0x28
#define FDD_CMD_MKDIR       0x29
#define FDD_CMD_MOTOR_ON    0x40
#define FDD_CMD_MOTOR_OFF   0x41

#define FDD_ST_OK           0x00
#define FDD_ST_ERROR        0x01
#define FDD_ST_NOTREADY     0x02
#define FDD_ST_NOTFOUND     0x03
#define FDD_ST_EOF          0x04
#define FDD_ST_READONLY     0x05
#define FDD_ST_EXISTS       0x06
#define FDD_ST_INVALID      0x07
#define FDD_ST_DENIED       0x08
#define FDD_ST_BUSY         0xFF

#define FDD_RX_MAX          32

typedef struct {
    uint8_t drive_type;
    uint8_t have_disk;
    uint8_t write_protected;
    uint8_t motor;
    uint8_t selected_drive;
} FddStatus;

/* Map Arduino status → vinxru lastError codes (fs.h) */
uint8_t fdd_map_error(uint8_t st);

void    fdd_init(void);
uint8_t fdd_get_status(FddStatus *st);
uint8_t fdd_motor_on(void);
uint8_t fdd_motor_off(void);

uint8_t fdd_dir_start(const char *path);
uint8_t fdd_dir_next(uint8_t *entry19); /* status+name14+attr+size4, or 1 byte EOF */

uint8_t fdd_open_read(const char *path);
uint8_t fdd_open_write(const char *path);
uint8_t fdd_close(void);
uint8_t fdd_delete(const char *path);
uint8_t fdd_mkdir(const char *path);
uint8_t fdd_seek(uint32_t offset);
uint8_t fdd_get_size(uint32_t *size);
uint8_t fdd_tell(uint32_t *pos);

/* Read up to max_len (<=30); *out_len actual; returns FDD_ST_* */
uint8_t fdd_read(uint8_t *dst, uint8_t max_len, uint8_t *out_len);
uint8_t fdd_write(const uint8_t *src, uint8_t len);

#endif
