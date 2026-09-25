// -----------------------------------------------------------------------------
// 3.5"/5.25" DD/HD Disk controller for Arduino
// Copyright (C) 2021 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
//
// I2C slave (адрес 0x42) для контроллера Радио-86РК — palmira original software.
// База: https://github.com/dhansel/ArduinoFDC
// -----------------------------------------------------------------------------

#include "ArduinoFDC.h"
#include "ff.h"
#include <Wire.h>
// -------------------------------------------------------------------------------------------------
// I2C Slave interface
// -------------------------------------------------------------------------------------------------

#define I2C_SLAVE_ADDR 0x42

#define I2C_BUFFER_SIZE 32
volatile byte i2c_rx_buffer[I2C_BUFFER_SIZE];
volatile byte i2c_rx_len = 0;
volatile byte i2c_tx_buffer[I2C_BUFFER_SIZE];
volatile byte i2c_tx_len = 0;
volatile bool i2c_new_cmd = false;
volatile bool i2c_busy = false;
volatile byte i2c_cmd = 0;
volatile byte i2c_args[I2C_BUFFER_SIZE - 1];
volatile byte i2c_args_len = 0;

// Multi-transfer sector I/O state
volatile int  i2c_sector_offset = -1; // -1 = inactive, 0..511 = position in sector buffer
volatile byte i2c_sector_track = 0;
volatile byte i2c_sector_side  = 0;
volatile byte i2c_sector_num   = 0;
volatile bool i2c_sector_write_mode = false; // true = accumulating data for write
/* Sector bytes live in FatFs.win (defined with FatFs below). */

// I2C command codes
#define CMD_SELECT_DRIVE  0x01
#define CMD_GET_STATUS    0x02
#define CMD_DIR_START     0x10
#define CMD_DIR_NEXT      0x11
#define CMD_FILE_OPEN_READ  0x20
#define CMD_FILE_READ     0x21
#define CMD_FILE_OPEN_WRITE 0x22
#define CMD_FILE_WRITE    0x23
#define CMD_FILE_CLOSE    0x24
#define CMD_FILE_DELETE   0x25
#define CMD_FILE_SEEK     0x26
#define CMD_FILE_SIZE     0x27
#define CMD_FILE_TELL     0x28
#define CMD_MKDIR         0x29
#define CMD_FORMAT        0x30
#define CMD_GET_DISK_INFO 0x31
#define CMD_MOTOR_ON      0x40
#define CMD_MOTOR_OFF     0x41
#define CMD_READ_SECTOR   0x50
#define CMD_WRITE_SECTOR  0x51
#define CMD_SECTOR_DATA   0x52

// I2C status codes
#define I2C_STATUS_OK       0x00
#define I2C_STATUS_ERROR    0x01
#define I2C_STATUS_NOTREADY 0x02
#define I2C_STATUS_NOTFOUND 0x03
#define I2C_STATUS_EOF      0x04
#define I2C_STATUS_READONLY 0x05
#define I2C_STATUS_EXISTS   0x06
#define I2C_STATUS_INVALID  0x07
#define I2C_STATUS_DENIED   0x08
#define I2C_STATUS_MOREDATA 0x10
#define I2C_STATUS_BUSY     0xFF

// comment this out to remove high-level ArduDOS functions
#define USE_ARDUDOS

// commenting this out will remove the low-level disk monitor
//#define USE_MONITOR

// comenting this out will remove support for XModem data transfers
//#define USE_XMODEM


// Note: XModem and MONITOR are disabled to save flash

// -------------------------------------------------------------------------------------------------
// Basic helper functions
// -------------------------------------------------------------------------------------------------

#define TEMPBUFFER_SIZE 80
byte tempbuffer[TEMPBUFFER_SIZE];

unsigned long motor_timeout = 0;


void print_hex(byte b)
{
  if( b<16 ) Serial.write('0');
  Serial.print(b, HEX);
}


void dump_buffer(int offset, byte *buf, int n)
{
  int i = 0;
  while( i<n )
    {
      print_hex((offset+i)/256); 
      print_hex((offset+i)&255); 
      Serial.write(':');

      for(int j=0; j<16; j++)
        {
          if( (j&7)==0  ) Serial.write(' ');
          if( i+j<n ) print_hex(buf[i+j]); else Serial.print(F("  "));
          Serial.write(' ');
        }

      Serial.write(' ');
      for(int j=0; j<16; j++)
        {
          if( (j&7)==0  ) Serial.write(' ');
          if( i+j<n ) Serial.write(isprint(buf[i+j]) ? buf[i+j] : '.'); else Serial.write(' ');
        }

      Serial.println();
      i += 16;
    }
}


// Non-blocking serial line reader
// Returns NULL if line not complete yet, or pointer to buffer on complete line
char *read_user_cmd_nb(void *buffer, int buflen)
{
  char *buf = (char *) buffer;
  static byte l = 0;
  
  while( Serial.available() )
    {
      int i = Serial.read();

      if( i==13 || i==10 )
        { 
          Serial.println(); 
          // Finalize: trim trailing whitespace
          while( l>0 && isspace(buf[l-1]) ) l--;
          buf[l] = 0;
          l = 0; // reset for next line
          return buf;
        }
      else if( i==27 )
        { 
          l=0; 
          Serial.println(); 
          buf[0] = 0;
          l = 0;
          return buf;
        }
      else if( i==8 )
        { 
          if( l>0 )
            { Serial.write(8); Serial.write(' '); Serial.write(8); l--; }
        }
      else if( isprint(i) && l<buflen-1 )
        { buf[l++] = i; Serial.write(i); }
    }

  return NULL; // line not complete yet
}


bool confirm_formatting()
{
  int c;
  Serial.print(F("Formatting will erase all data on the disk in drive "));
  Serial.write('A' + ArduinoFDC.selectedDrive());
  Serial.print(F(". Continue (y/n)?"));
  while( (c=Serial.read())<0 );
  do { delay(1); } while( Serial.read()>=0 );
  Serial.println();
  return c=='y';
}


void print_drive_type(byte n)
{
  switch( n )
    {
    case ArduinoFDCClass::DT_5_DD: Serial.print(F("5.25\" DD")); break;
    case ArduinoFDCClass::DT_5_DDonHD: Serial.print(F("5.25\" DD disk in HD drive")); break;
    case ArduinoFDCClass::DT_5_HD: Serial.print(F("5.25\" HD")); break;
    case ArduinoFDCClass::DT_3_DD: Serial.print(F("3.5\" DD")); break;
    case ArduinoFDCClass::DT_3_HD: Serial.print(F("3.5\" HD")); break;
    default: Serial.print(F("Unknown"));
    }
}


void print_error(byte n)
{
  Serial.print(F("Error: "));
  switch( n )
    {
    case S_OK        : Serial.print(F("No error")); break;
    case S_NOTINIT   : Serial.print(F("ArduinoFDC.begin() was not called")); break;
    case S_NOTREADY  : Serial.print(F("Drive not ready")); break;
    case S_NOSYNC    : Serial.print(F("No sync marks found")); break;
    case S_NOHEADER  : Serial.print(F("Sector header not found")); break;
    case S_INVALIDID : Serial.print(F("Data record has unexpected id")); break;
    case S_CRC       : Serial.print(F("Data checksum error")); break;
    case S_NOTRACK0  : Serial.print(F("No track 0 signal detected")); break;
    case S_VERIFY    : Serial.print(F("Verify after write failed")); break;
    case S_READONLY  : Serial.print(F("Disk is write protected")); break;
    default          : Serial.print(F("Unknonwn error")); break;
    }
  Serial.println('!');
}


void set_drive_type(int n)
{
  ArduinoFDC.setDriveType((ArduinoFDCClass::DriveType) n);
  Serial.print(F("Setting disk type for drive ")); Serial.write('A'+ArduinoFDC.selectedDrive());
  Serial.print(F(" to ")); print_drive_type(ArduinoFDC.getDriveType());
  Serial.println();
}


// -------------------------------------------------------------------------------------------------
// High-level ArduDOS 
// -------------------------------------------------------------------------------------------------


///#ifdef USE_ARDUDOS

static FATFS FatFs;
static FIL   FatFsFile;
#define I2C_sector_buffer FatFs.win



// Some versions of Arduino appear to have problems with the FRESULT type in the print_ff_error
// function - which reportedly can be fixed by putting a forward declaration here (thanks to rtrimbler!).
// I am not able to reproduce the error but adding a forward declaration can't hurt.
void print_ff_error(FRESULT fr);

void print_ff_error(FRESULT fr)
{
  Serial.print(F("Error #")); 
  Serial.print(fr);
  Serial.print(F(": "));
  switch( fr )
    {
    case FR_DISK_ERR: Serial.print(F("Low-level disk error")); break;
    case FR_INT_ERR: Serial.print(F("Internal error")); break;
    case FR_NOT_READY: Serial.print(F("Drive not ready")); break;
    case FR_NO_FILE: Serial.print(F("File not found")); break;
    case FR_NO_PATH: Serial.print(F("Path not found")); break;
    case FR_INVALID_NAME: Serial.print(F("Invalid path format")); break;
    case FR_DENIED: Serial.print(F("Directory full")); break;
    case FR_EXIST: Serial.print(F("File exists")); break;
    case FR_INVALID_OBJECT: Serial.print(F("Invalid object")); break;
    case FR_WRITE_PROTECTED: Serial.print(F("Disk is write protected")); break;
    case FR_INVALID_DRIVE: Serial.print(F("Invalid drive")); break;
    case FR_NOT_ENABLED: Serial.print(F("The volume has no work area")); break;
    case FR_NO_FILESYSTEM: Serial.print(F("Not a FAT file system")); break;
    case FR_MKFS_ABORTED: Serial.print(F("Format aborted due to error")); break;
    case FR_NOT_ENOUGH_CORE: Serial.print(F("Out of memory")); break;
    case FR_INVALID_PARAMETER: Serial.print(F("Invalid parameter")); break;
    default: Serial.print(F("Unknown")); break;
    }
  Serial.println();
}


// -------------------------------------------------------------------------------------------------
// Non-blocking ArduDOS state machine
// -------------------------------------------------------------------------------------------------

enum ArduDOS_State {
  ADS_IDLE,
  ADS_PROMPT,
  ADS_CMD_READY,
  ADS_PROCESS,
  ADS_WRITE_LINE,
  ADS_CONFIRM_FORMAT
};

static ArduDOS_State arduDOS_state = ADS_IDLE;
static char *arduDOS_cmd = NULL;

void arduDOS_process_cmd(const char *cmd)
{
  UINT count;
  FRESULT fr;

  if( ArduinoFDC.diskChanged() )
    {
      Serial.println("Disk change detected!");
      ArduinoFDC.setDriveType(ArduinoFDCClass::DT_3_HD);
      f_mount(&FatFs, "0:", 0);
    }

  if( strcmp_PF(cmd, PSTR("a:"))==0 || strcmp_PF(cmd, PSTR("b:"))==0 )
    {
      byte drive = cmd[0]-'a';
      if( drive != ArduinoFDC.selectedDrive() )
        {
          ArduinoFDC.motorOff();
          motor_timeout = 0;
          ArduinoFDC.selectDrive(drive);
        }

      f_mount(&FatFs, "0:", 0);
    }
  else if( strncmp_P(cmd, PSTR("dir"), 3)==0 )
    {
      DIR dir;
      FILINFO finfo;

      ArduinoFDC.motorOn();
      fr = f_opendir(&dir, strlen(cmd)<5 ? "0:\\" : cmd+4);
      if (fr == FR_OK) 
        {
          count = 0;
          while(1)
            {
              fr = f_readdir(&dir, &finfo);
              if( fr!=FR_OK || finfo.fname[0]==0 )
                break;
              
              char *c = finfo.fname;
              byte col = 0;
              while( *c!=0 && *c!='.' ) { Serial.write(toupper(*c)); col++; c++; }
              while( col<9 ) { Serial.write(' '); col++; }
              if( *c=='.' )
                {
                  c++;
                  while( *c!=0 ) { Serial.write(toupper(*c)); col++; c++; }
                }
              while( col<14 ) { Serial.write(' '); col++; }
              if( finfo.fattrib & AM_DIR )
                Serial.println(F("<DIR>"));
              else
                Serial.println(finfo.fsize);
              count++;
            }

          f_closedir(&dir);

          if( fr==FR_OK )
            {
              if( count==0 ) Serial.println(F("No files."));
              
              FATFS *fs;
              DWORD fre_clust;
              fr = f_getfree("0:", &fre_clust, &fs);

              if( fr==FR_OK )
                { Serial.print(fre_clust * fs->csize * 512); Serial.println(F(" bytes free.")); }
            }

          if( fr!=FR_OK )
            print_ff_error(fr);
        }
      else
        print_ff_error(fr);
    }
  else if( strncmp_P(cmd, PSTR("type "), 5)==0 )
    {
      ArduinoFDC.motorOn();
      fr = f_open(&FatFsFile, cmd+5, FA_READ);
      if( fr == FR_OK )
        {
          count = 1;
          while( count>0 )
            {
              fr = f_read(&FatFsFile, tempbuffer, TEMPBUFFER_SIZE, &count);
              if( fr == FR_OK )
                Serial.write(tempbuffer, count);
              else
                print_ff_error(fr);
            }
          f_close(&FatFsFile);
        }
      else
        print_ff_error(fr);
    }
  else if( strncmp_P(cmd, PSTR("dump "), 5)==0 )
    {
      ArduinoFDC.motorOn();
      fr = f_open(&FatFsFile, cmd+5, FA_READ);
      if( fr == FR_OK )
        {
          count = 1;
          int offset = 0;
          while( count>0 )
            {
              fr = f_read(&FatFsFile, tempbuffer, (TEMPBUFFER_SIZE/16)*16, &count);
              if( fr == FR_OK )
                { dump_buffer(offset, tempbuffer, count); offset += count; }
              else
                print_ff_error(fr);
            }
          f_close(&FatFsFile);
        }
      else
        print_ff_error(fr);
    }
  else if( strncmp_P(cmd, PSTR("del "), 4)==0 )
    {
      ArduinoFDC.motorOn();
      fr = f_unlink(cmd+4);
      if( fr != FR_OK )
        print_ff_error(fr);
    }
  else if( strncmp_P(cmd, PSTR("mkdir "), 6)==0 )
    {
      ArduinoFDC.motorOn();
      fr = f_mkdir(cmd+6);
      if( fr != FR_OK )
        print_ff_error(fr);
    }
  else if( strncmp_P(cmd, PSTR("rmdir "), 6)==0 )
    {
      ArduinoFDC.motorOn();
      fr = f_rmdir(cmd+6);
      if( fr != FR_OK )
        print_ff_error(fr);
    }
  else if( strncmp_P(cmd, PSTR("disktype "), 9)==0 )
    {
      set_drive_type(atoi(cmd+9));
      f_mount(&FatFs, "0:", 0);
    }
  else if( strncmp_P(cmd, PSTR("format"), 6)==0 )
    {
      MKFS_PARM param;
      param.fmt = FM_FAT | FM_SFD; // FAT12 type, no disk partitioning
      param.n_fat = 2;             // number of FATs
      param.n_heads = 2;           // number of heads
      param.n_sec_track = ArduinoFDC.numSectors(); 
      param.align = 1;             // block alignment (not used for FAT12)
      
      switch( ArduinoFDC.getDriveType() )
        {
        case ArduinoFDCClass::DT_5_DD:
        case ArduinoFDCClass::DT_5_DDonHD:
          param.au_size = 1024; // bytes/cluster
          param.n_root  = 112;  // number of root directory entries
          param.media   = 0xFD; // media descriptor
          break;
          
        case ArduinoFDCClass::DT_5_HD:
          param.au_size = 512;  // bytes/cluster
          param.n_root  = 224;  // number of root directory entries
          param.media   = 0xF9; // media descriptor
          break;

        case ArduinoFDCClass::DT_3_DD:
          param.au_size = 1024; // bytes/cluster
          param.n_root  = 112;  // number of root directory entries
          param.media   = 0xF9; // media descriptor
          break;

        case ArduinoFDCClass::DT_3_HD:
          param.au_size = 512;  // bytes/cluster
          param.n_root  = 224;  // number of root directory entries
          param.media   = 0xF0; // media descriptor
          break;
        }
      
      if( confirm_formatting() )
        {
          byte st;
          ArduinoFDC.motorOn();
          f_unmount("0:");
          if( strstr(cmd, "/q") || (st=ArduinoFDC.formatDisk(FatFs.win))==S_OK )
            {
              Serial.println(F("Initializing file system...\n"));
              FRESULT fr = f_mkfs ("0:", &param, FatFs.win, 512);
              if( fr != FR_OK ) print_ff_error(fr);
            }
          else
            print_error(st);
          
          f_mount(&FatFs, "0:", 0);
        }
    }
  else if( strcmp_P(cmd, PSTR("help"))==0 || strcmp_P(cmd, PSTR("h"))==0 || strcmp_P(cmd, PSTR("?"))==0 )
    {
      Serial.print(F("Valid commands: dir, type, dump, del, mkdir, rmdir, disktype, format, help"));
      Serial.println();
    }
  else if( cmd[0]!=0 )
    {
      Serial.print(F("Unknown command: ")); 
      Serial.print(cmd);
    }
}

void arduDOS_nb()
{
  switch( arduDOS_state )
  {
    case ADS_IDLE:
      // Check if serial data is available to start a new command
      if( Serial.available() )
      {
        Serial.println();
        Serial.write('A'+ArduinoFDC.selectedDrive());
        Serial.print(F(":>"));
        arduDOS_state = ADS_PROMPT;
      }
      break;

    case ADS_PROMPT:
      // Read one line non-blocking
      {
        char *line = read_user_cmd_nb(tempbuffer, TEMPBUFFER_SIZE);
        if( line != NULL )
        {
          arduDOS_cmd = line;
          arduDOS_state = ADS_CMD_READY;
        }
      }
      break;

    case ADS_CMD_READY:
      // Process the command
      arduDOS_process_cmd(arduDOS_cmd);
      motor_timeout = millis() + 5000;
      arduDOS_state = ADS_IDLE;
      break;

    default:
      arduDOS_state = ADS_IDLE;
      break;
  }
}


// -------------------------------------------------------------------------------------------------
// I2C receive handler (master -> slave)
void i2c_receive(int len)
{
  byte i;
  for(i=0; i<len && i<I2C_BUFFER_SIZE; i++)
    i2c_rx_buffer[i] = Wire.read();
  i2c_rx_len = i;
  
  if( i2c_rx_len >= 1 )
  {
    i2c_cmd = i2c_rx_buffer[0];
    i2c_args_len = i2c_rx_len - 1;
    if( i2c_args_len > (I2C_BUFFER_SIZE - 1) )
      i2c_args_len = I2C_BUFFER_SIZE - 1;
    for(byte j=0; j<i2c_args_len; j++)
      i2c_args[j] = i2c_rx_buffer[1+j];
    i2c_busy = true;
    i2c_new_cmd = true;
  }
}

// I2C request handler (master <- slave)
void i2c_request()
{
  // Master may poll before loop() finishes FatFs/FDD work
  if( i2c_busy || i2c_new_cmd )
  {
    /* STM8 always clocks 32 bytes. A short reply wedges the TWI slave. */
    byte pad[32];
    for(byte i = 0; i < 32; i++) pad[i] = I2C_STATUS_BUSY;
    Wire.write(pad, 32);
    return;
  }

  // Continue sector read only after previous chunk was consumed (tx_len==0)
  if( i2c_tx_len == 0 && i2c_sector_offset >= 0 && i2c_sector_offset < 512
      && !i2c_sector_write_mode )
  {
    byte chunk_size = 30;
    int remaining = 512 - i2c_sector_offset;
    if( chunk_size > remaining ) chunk_size = remaining;
    
    i2c_tx_buffer[0] = (remaining > chunk_size) ? I2C_STATUS_MOREDATA : I2C_STATUS_OK;
    for(byte j=0; j<chunk_size; j++)
      i2c_tx_buffer[1+j] = I2C_sector_buffer[1 + i2c_sector_offset + j];
    i2c_tx_len = 1 + chunk_size;
    
    i2c_sector_offset += chunk_size;
    if( i2c_sector_offset >= 512 )
      i2c_sector_offset = -1;
  }

  {
    byte pad[32];
    byte n = i2c_tx_len;
    byte i;
    if( n > 32 ) n = 32;
    for(i = 0; i < 32; i++)
      pad[i] = (i < n) ? i2c_tx_buffer[i] : I2C_STATUS_BUSY;
    Wire.write(pad, 32);
    if( n > 0 )
      i2c_tx_len = 0;
  }
}


// -------------------------------------------------------------------------------------------------
// I2C command dispatcher
// -------------------------------------------------------------------------------------------------

static DIR   I2C_Dir;
static FILINFO I2C_Finfo;
static bool  I2C_dir_open = false;
void i2c_prepare_status(byte status)
{
  i2c_tx_buffer[0] = status;
  i2c_tx_len = 1;
}

void i2c_prepare_data(const byte *data, byte len)
{
  if( len > I2C_BUFFER_SIZE ) len = I2C_BUFFER_SIZE;
  for(byte i=0; i<len; i++)
    i2c_tx_buffer[i] = data[i];
  i2c_tx_len = len;
}

void i2c_process_command()
{
  FRESULT fr;
  byte status = I2C_STATUS_OK;
  
  // By default, clear TX buffer
  i2c_tx_len = 0;
  
  // If we get any command other than CMD_SECTOR_DATA while a sector write is pending,
  // abort the pending write
  if( i2c_cmd != CMD_SECTOR_DATA )
  {
    i2c_sector_offset = -1;
    i2c_sector_write_mode = false;
  }
  
  switch( i2c_cmd )
  {
    case CMD_SELECT_DRIVE:
    {
      if( i2c_args_len >= 1 )
      {
        byte drive = i2c_args[0];
        if( drive < 2 )
        {
          ArduinoFDC.motorOff();
          ArduinoFDC.selectDrive(drive);
          f_mount(&FatFs, "0:", 0);
        }
        else
          status = I2C_STATUS_INVALID;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_GET_STATUS:
    {
      /* Do NOT call haveDisk() — it spins motor + INDEX wait (~1s) and
       * leaves Radio-86RK stuck on ERR_WAIT (0x41). */
      i2c_tx_buffer[0] = I2C_STATUS_OK;
      i2c_tx_buffer[1] = (byte) ArduinoFDC.getDriveType();
      i2c_tx_buffer[2] = 1; /* assume present; open/read report real errors */
      i2c_tx_buffer[3] = ArduinoFDC.isWriteProtected() ? 1 : 0;
      i2c_tx_buffer[4] = ArduinoFDC.motorRunning() ? 1 : 0;
      i2c_tx_buffer[5] = ArduinoFDC.selectedDrive();
      i2c_tx_len = 6;
      return;
    }
    
    case CMD_DIR_START:
    {
      I2C_dir_open = false;
      ArduinoFDC.setDriveType(ArduinoFDCClass::DT_3_HD);
      ArduinoFDC.motorOn();
      f_mount(&FatFs, "0:", 0);
      
      if( i2c_args_len >= 1 )
      {
        if( i2c_args_len < (I2C_BUFFER_SIZE - 1) )
          i2c_args[i2c_args_len] = 0;
        else
          i2c_args[I2C_BUFFER_SIZE - 2] = 0;
      }
      const char *path = (i2c_args_len >= 1 && i2c_args[0] != 0) ? (const char *)i2c_args : "0:\\";
      fr = f_opendir(&I2C_Dir, path);
      if( fr == FR_OK )
      {
        I2C_dir_open = true;
        status = I2C_STATUS_OK;
      }
      else
        status = I2C_STATUS_NOTFOUND;
      break;
    }
    
    case CMD_DIR_NEXT:
    {
      if( I2C_dir_open )
      {
        fr = f_readdir(&I2C_Dir, &I2C_Finfo);
        if( fr == FR_OK && I2C_Finfo.fname[0] != 0 )
        {
          // Pack entry: name (null-terminated) + attrib + size (4 bytes LE)
          byte *p = i2c_tx_buffer;
          byte i = 0;
          // Copy filename (up to 12 chars + null)
          while( I2C_Finfo.fname[i] && i < 12 )
          {
            p[1+i] = toupper(I2C_Finfo.fname[i]);
            i++;
          }
          p[1+i] = 0; // null-terminate
          p[1+13] = I2C_Finfo.fattrib;
          // Size as 4-byte LE
          unsigned long fsize = I2C_Finfo.fsize;
          p[1+14] = fsize & 0xFF;
          p[1+15] = (fsize >> 8) & 0xFF;
          p[1+16] = (fsize >> 16) & 0xFF;
          p[1+17] = (fsize >> 24) & 0xFF;
          p[0] = I2C_STATUS_OK;
          i2c_tx_len = 1 + 14 + 4; // status + name(14) + attrib + size
        }
        else
        {
          // No more entries
          f_closedir(&I2C_Dir);
          I2C_dir_open = false;
          i2c_tx_buffer[0] = I2C_STATUS_EOF;
          i2c_tx_len = 1;
        }
        return; // already set buffer
      }
      else
      {
        i2c_tx_buffer[0] = I2C_STATUS_EOF;
        i2c_tx_len = 1;
        return;
      }
    }
    
    case CMD_FILE_OPEN_READ:
    {
      if( i2c_args_len >= 1 )
      {
        if( i2c_args_len < (I2C_BUFFER_SIZE - 1) )
          i2c_args[i2c_args_len] = 0;
        else
          i2c_args[I2C_BUFFER_SIZE - 2] = 0;
        ArduinoFDC.motorOn();
        /* Same path as the serial "dir" command. No geometry probe. */
        ArduinoFDC.setDriveType(ArduinoFDCClass::DT_3_HD);
        f_mount(&FatFs, "0:", 0);
        fr = f_open(&FatFsFile, (const char *)i2c_args, FA_READ);
        if( fr == FR_OK )
          status = I2C_STATUS_OK;
        else if( fr == FR_NO_FILE || fr == FR_NO_PATH )
          status = I2C_STATUS_NOTFOUND;
        else
          status = (byte)(0x10 | (fr & 0x0F));
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_FILE_READ:
    {
      if( i2c_args_len >= 1 )
      {
        UINT br;
        byte read_len = i2c_args[0];
        if( read_len > 30 ) read_len = 30; // max 30 bytes data + 1 status + 1 actual len
        fr = f_read(&FatFsFile, (void *)(i2c_tx_buffer+2), read_len, &br);
        if( fr == FR_OK )
        {
          i2c_tx_buffer[0] = I2C_STATUS_OK;
          i2c_tx_buffer[1] = (byte) br;
          i2c_tx_len = 2 + br;
          if( br < read_len )
            i2c_tx_buffer[0] = I2C_STATUS_EOF; // EOF reached
        }
        else
        {
          i2c_tx_buffer[0] = (byte)(0x10 | (fr & 0x0F));
          i2c_tx_len = 1;
        }
        return;
      }
      else
      {
        i2c_tx_buffer[0] = I2C_STATUS_INVALID;
        i2c_tx_len = 1;
        return;
      }
    }
    
    case CMD_FILE_OPEN_WRITE:
    {
      if( i2c_args_len >= 1 )
      {
        if( i2c_args_len < (I2C_BUFFER_SIZE - 1) )
          i2c_args[i2c_args_len] = 0;
        else
          i2c_args[I2C_BUFFER_SIZE - 2] = 0;
        ArduinoFDC.motorOn();
        fr = f_open(&FatFsFile, (const char *)i2c_args, FA_WRITE | FA_CREATE_ALWAYS);
        if( fr == FR_OK )
          status = I2C_STATUS_OK;
        else if( fr == FR_DENIED )
          status = I2C_STATUS_DENIED;
        else
          status = I2C_STATUS_ERROR;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_FILE_WRITE:
    {
      if( i2c_args_len >= 1 )
      {
        UINT bw;
        // Data to write is in i2c_args[1..len-1], byte count = i2c_args[0]
        byte write_len = i2c_args[0];
        if( write_len > 30 ) write_len = 30;
        fr = f_write(&FatFsFile, (const void *)(i2c_args+1), write_len, &bw);
        if( fr == FR_OK )
          status = I2C_STATUS_OK;
        else
          status = I2C_STATUS_ERROR;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_FILE_CLOSE:
    {
      f_close(&FatFsFile);
      status = I2C_STATUS_OK;
      break;
    }
    
    case CMD_FILE_DELETE:
    {
      if( i2c_args_len >= 1 )
      {
        if( i2c_args_len < (I2C_BUFFER_SIZE - 1) )
          i2c_args[i2c_args_len] = 0;
        else
          i2c_args[I2C_BUFFER_SIZE - 2] = 0;
        fr = f_unlink((const char *)i2c_args);
        if( fr == FR_OK )
          status = I2C_STATUS_OK;
        else if( fr == FR_NO_FILE )
          status = I2C_STATUS_NOTFOUND;
        else
          status = I2C_STATUS_ERROR;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_FILE_SEEK:
    {
      if( i2c_args_len >= 4 )
      {
        unsigned long offset = i2c_args[0] | ((unsigned long)i2c_args[1] << 8) |
                               ((unsigned long)i2c_args[2] << 16) | ((unsigned long)i2c_args[3] << 24);
        fr = f_lseek(&FatFsFile, offset);
        if( fr == FR_OK )
          status = I2C_STATUS_OK;
        else
          status = I2C_STATUS_ERROR;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }

    case CMD_FILE_SIZE:
    {
      unsigned long fsize = f_size(&FatFsFile);
      i2c_tx_buffer[0] = I2C_STATUS_OK;
      i2c_tx_buffer[1] = fsize & 0xFF;
      i2c_tx_buffer[2] = (fsize >> 8) & 0xFF;
      i2c_tx_buffer[3] = (fsize >> 16) & 0xFF;
      i2c_tx_buffer[4] = (fsize >> 24) & 0xFF;
      i2c_tx_len = 5;
      return;
    }

    case CMD_FILE_TELL:
    {
      unsigned long pos = f_tell(&FatFsFile);
      i2c_tx_buffer[0] = I2C_STATUS_OK;
      i2c_tx_buffer[1] = pos & 0xFF;
      i2c_tx_buffer[2] = (pos >> 8) & 0xFF;
      i2c_tx_buffer[3] = (pos >> 16) & 0xFF;
      i2c_tx_buffer[4] = (pos >> 24) & 0xFF;
      i2c_tx_len = 5;
      return;
    }

    case CMD_MKDIR:
    {
      if( i2c_args_len >= 1 )
      {
        if( i2c_args_len < (I2C_BUFFER_SIZE - 1) )
          i2c_args[i2c_args_len] = 0;
        else
          i2c_args[I2C_BUFFER_SIZE - 2] = 0;
        fr = f_mkdir((const char *)i2c_args);
        if( fr == FR_OK )
          status = I2C_STATUS_OK;
        else if( fr == FR_EXIST )
          status = I2C_STATUS_EXISTS;
        else
          status = I2C_STATUS_ERROR;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_FORMAT:
    {
      ArduinoFDC.motorOn();
      byte fmt_status = ArduinoFDC.formatDisk((byte *)I2C_sector_buffer, 
                        i2c_args_len >= 1 ? i2c_args[0] : 0,
                        i2c_args_len >= 2 ? i2c_args[1] : 255);
      if( fmt_status == S_OK )
        status = I2C_STATUS_OK;
      else
        status = I2C_STATUS_ERROR;
      break;
    }
    
    case CMD_GET_DISK_INFO:
    {
      // Returns: numHeads, numTracks, numSectors, driveType
      i2c_tx_buffer[0] = I2C_STATUS_OK;
      i2c_tx_buffer[1] = ArduinoFDC.numHeads();
      i2c_tx_buffer[2] = ArduinoFDC.numTracks();
      i2c_tx_buffer[3] = ArduinoFDC.numSectors();
      i2c_tx_buffer[4] = (byte) ArduinoFDC.getDriveType();
      i2c_tx_len = 5;
      return;
    }
    
    case CMD_MOTOR_ON:
    {
      ArduinoFDC.motorOn();
      status = I2C_STATUS_OK;
      break;
    }
    
    case CMD_MOTOR_OFF:
    {
      ArduinoFDC.motorOff();
      status = I2C_STATUS_OK;
      break;
    }
    
    case CMD_READ_SECTOR:
    {
      if( i2c_args_len >= 3 )
      {
        ArduinoFDC.motorOn();
        byte track = i2c_args[0];
        byte side  = i2c_args[1];
        byte sector = i2c_args[2];
        byte rd_status = ArduinoFDC.readSector(track, side, sector, (byte *)I2C_sector_buffer);
        if( rd_status == S_OK )
        {
          // Set up multi-transfer sector read
          // First chunk will be sent on next i2c_request()
          i2c_sector_offset = 0;
          i2c_sector_write_mode = false;
          // Prepare first chunk immediately for the first read request
          byte chunk_size = 30;
          i2c_tx_buffer[0] = I2C_STATUS_MOREDATA;
          for(int j=0; j<chunk_size; j++)
            i2c_tx_buffer[1+j] = I2C_sector_buffer[1+j];
          i2c_tx_len = 1 + chunk_size;
          i2c_sector_offset = chunk_size;
          // Note: if exactly 512 bytes, last chunk will have I2C_STATUS_OK
          // The i2c_request() handler will continue delivering chunks
        }
        else
        {
          i2c_sector_offset = -1;
          i2c_tx_buffer[0] = I2C_STATUS_ERROR;
          i2c_tx_len = 1;
        }
        return;
      }
      else
      {
        i2c_sector_offset = -1;
        i2c_tx_buffer[0] = I2C_STATUS_INVALID;
        i2c_tx_len = 1;
        return;
      }
    }
    
    case CMD_WRITE_SECTOR:
    {
      if( i2c_args_len >= 3 )
      {
        ArduinoFDC.motorOn();
        i2c_sector_track = i2c_args[0];
        i2c_sector_side  = i2c_args[1];
        i2c_sector_num   = i2c_args[2];
        // Set up multi-transfer sector write: accumulate data from subsequent writes
        i2c_sector_offset = 0;
        i2c_sector_write_mode = true;
        status = I2C_STATUS_OK;
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    case CMD_SECTOR_DATA:
    {
      // Continuation data for pending sector write
      if( i2c_sector_write_mode && i2c_sector_offset >= 0 && i2c_sector_offset < 512 )
      {
        byte data_len = i2c_args_len;
        if( data_len > (512 - i2c_sector_offset) )
          data_len = 512 - i2c_sector_offset;
        for(byte j=0; j<data_len; j++)
          I2C_sector_buffer[1 + i2c_sector_offset + j] = i2c_args[j];
        i2c_sector_offset += data_len;
        
        // Check if sector is complete
        if( i2c_sector_offset >= 512 )
        {
          // Commit the write
          byte wr_status = ArduinoFDC.writeSector(i2c_sector_track, i2c_sector_side, i2c_sector_num, (byte *)I2C_sector_buffer, true);
          i2c_sector_offset = -1;
          i2c_sector_write_mode = false;
          if( wr_status == S_OK )
            status = I2C_STATUS_OK;
          else
            status = I2C_STATUS_ERROR;
        }
        else
        {
          // More data needed
          status = I2C_STATUS_OK;
        }
      }
      else
        status = I2C_STATUS_INVALID;
      break;
    }
    
    default:
      status = I2C_STATUS_INVALID;
      break;
  }
  
  i2c_prepare_status(status);
}

// -------------------------------------------------------------------------------------------------
// Main functions
// -------------------------------------------------------------------------------------------------


void setup() 
{
  Serial.begin(115200);
  
  // Try to auto-detect drive types:
  // Start with 3.5" HD as default, then probe drives
  ArduinoFDC.begin(ArduinoFDCClass::DT_3_HD, ArduinoFDCClass::DT_3_HD);
  ArduinoFDC.selectDrive(0);

  // Initialize I2C slave
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_request);

  // Mount FAT FS for I2C use
  f_mount(&FatFs, "0:", 0);

  // must save flash space if all three of ARDUDOS/MONITOR/XMODEM are enabled on UNO
#if defined(USE_ARDUDOS)
  Serial.print(F("Drive A: ")); print_drive_type(ArduinoFDC.getDriveType()); Serial.println();
  if( ArduinoFDC.selectDrive(1) )
    {
      Serial.print(F("Drive B: ")); print_drive_type(ArduinoFDC.getDriveType()); Serial.println();
      ArduinoFDC.selectDrive(0);
    }
#endif
}


void loop() 
{
  if( motor_timeout > 0 && (long)(millis() - motor_timeout) >= 0 )
  {
    ArduinoFDC.motorOff();
    motor_timeout = 0;
  }

  // Process I2C commands if any
  if( i2c_new_cmd )
  {
    i2c_new_cmd = false;
    motor_timeout = 0;
    i2c_process_command();
    i2c_busy = false;
    if( ArduinoFDC.motorRunning() )
      motor_timeout = millis() + 4000;
  }
  
  // Process serial ArduDOS commands (non-blocking)
  arduDOS_nb();
}


