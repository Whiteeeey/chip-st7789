// Wokwi st7789 Display Driver Chip
// Based on the following driver
// Wokwi IL9163 Display Driver Chip
//
// SPDX-License-Identifier: MIT
// Copyright (C) 2022 Uri Shaked / wokwi.com

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef enum {
  MODE_COMMAND = 0,
  MODE_DATA = 1,
} chip_mode_t;

typedef struct {
  pin_t    cs_pin;
  pin_t    dc_pin;
  pin_t    rst_pin;
  spi_dev_t spi;
  uint8_t  spi_buffer[2048];

  /* Framebuffer state */
  buffer_t framebuffer;
  uint32_t width;
  uint32_t height;

  /* Command state machine */
  chip_mode_t mode;
  uint8_t command_code;
  uint8_t command_size;
  uint8_t command_index;
  uint8_t command_buf[16];
  bool ram_write;

  // Memory and addressing settings
  uint32_t active_column;
  uint32_t active_page;
  uint32_t column_start;
  uint32_t column_end;
  uint32_t page_start;
  uint32_t page_end;
  uint32_t scanning_direction;
} chip_state_t;

/* Chip command codes */
#define CMD_NOP      (0x00)
#define CMD_SWRESET  (0x01)
#define CMD_SLPIN    (0x10)
#define CMD_SLPOUT   (0x11)
#define CMD_INVOFF   (0x20)
#define CMD_INVON    (0x21)
#define CMD_DISPOFF  (0x28)
#define CMD_DISPON   (0x29)
#define CMD_CASET    (0x2a)
#define CMD_RASET    (0x2b)
#define CMD_RAMWR    (0x2c)
#define CMD_MADCTL   (0x36)
#define CMD_COLMOD   (0x3a)
#define CMD_FRMCTR1  (0xb1)
#define CMD_FRMCTR2  (0xb2)
#define CMD_FRMCTR3  (0xb3)
#define CMD_INVCTR   (0xb4)
#define CMD_DISSET5  (0xb6)
#define CMD_PWCTR1   (0xc0)
#define CMD_PWCTR2   (0xc1)
#define CMD_PWCTR3   (0xc2)
#define CMD_PWCTR4   (0xc3)
#define CMD_PWCTR5   (0xc4)
#define CMD_VMCTR    (0xc5)
#define CMD_GMCTRP1  (0xe0)
#define CMD_GMCTRN1  (0xe1)

/* Scanning direction bits */
#define SCAN_MY (0b10000000)
#define SCAN_MX (0b01000000)
#define SCAN_MV (0b00100000)

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count);

void chip_reset(chip_state_t *chip) {
  chip->ram_write = false;
  chip->active_column = 0;
  chip->active_page = 0;
  chip->column_start = 0;
  chip->page_start = 0;
  // Use the framebuffer dimensions so address ranges match the actual display
  if (chip->width > 0 && chip->height > 0) {
    chip->column_end = chip->width - 1;
    chip->page_end = chip->height - 1;
  } else {
    chip->column_end = 127;
    chip->page_end = 127;
  }
  chip->scanning_direction = 0;
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip,
  };
  
  chip->cs_pin = pin_init("CS", INPUT_PULLUP);
  pin_watch(chip->cs_pin, &watch_config);

  chip->dc_pin = pin_init("DC", INPUT);
  pin_watch(chip->dc_pin, &watch_config);

  chip->rst_pin = pin_init("RST", INPUT_PULLUP);
  pin_watch(chip->rst_pin, &watch_config);

  const spi_config_t spi_config = {
    .sck = pin_init("SCL", INPUT),
    .mosi = pin_init("SDA", INPUT),
    .miso = NO_PIN,
    .done = chip_spi_done,
    .user_data = chip,
  };
  chip->spi = spi_init(&spi_config);

  // Initialize framebuffer (returns pointer inside buffer and fills width/height)
  chip->framebuffer = framebuffer_init(&chip->width, &chip->height);

  // default mode = command
  chip->mode = MODE_COMMAND;

  chip_reset(chip);
  
  printf("st7789 Driver Chip initialized! display %ux%u\n", chip->width, chip->height);
}

/* Convert 16-bit RGB565 to 32-bit RGBA (0xAARRGGBB) */
static inline uint32_t rgb565_to_rgba(uint16_t value) {
  uint32_t r5 = (value >> 11) & 0x1f;
  uint32_t g6 = (value >> 5) & 0x3f;
  uint32_t b5 = value & 0x1f;
  // expand to 8 bits per channel
  uint32_t r8 = (r5 << 3) | (r5 >> 2);
  uint32_t g8 = (g6 << 2) | (g6 >> 4);
  uint32_t b8 = (b5 << 3) | (b5 >> 2);
  return 0xff000000u | (r8 << 16) | (g8 << 8) | b8;
}

void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;

  // Handle CS pin logic
  if (pin == chip->cs_pin) {
    if (value == LOW) {
      // Selected: prepare to receive SPI
      chip->command_size = 0;
      chip->command_index = 0;
      chip->command_code = 0;
      spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
    } else {
      // Deselected: stop SPI and flush any pending
      spi_stop(chip->spi);
    }
  }

  // Handle DC pin logic: DC=0 -> command, DC=1 -> data
  if (pin == chip->dc_pin) {
    // Mode value equals pin value (0 or 1)
    chip_mode_t new_mode = value ? MODE_DATA : MODE_COMMAND;
    if (chip->mode != new_mode) {
      // Stop current SPI to process partial buffer
      spi_stop(chip->spi);
      chip->mode = new_mode;
      if (pin_read(chip->cs_pin) == LOW) {
        spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
      }
    }
  }

  if (pin == chip->rst_pin && value == LOW) {
    // hardware reset
    spi_stop(chip->spi);
    chip_reset(chip);
    // clear framebuffer to black
    if (chip->framebuffer) {
      uint32_t clear = 0xff000000u; // opaque black
      for (uint32_t i = 0; i < chip->width * chip->height; ++i) {
        buffer_write(chip->framebuffer, i * sizeof(clear), &clear, sizeof(clear));
      }
    }
  }
}

int command_args_size(uint8_t command_code) {
  switch (command_code) {
    case CMD_MADCTL:
    case CMD_PWCTR2:
    case CMD_INVCTR:
    case CMD_VMCTR:
    case CMD_COLMOD:  return 1;
    case CMD_PWCTR3:
    case CMD_PWCTR4:
    case CMD_PWCTR5:
    case CMD_DISSET5: return 2;
    case CMD_FRMCTR1:
    case CMD_FRMCTR2:
    case CMD_PWCTR1:  return 3;
    case CMD_CASET:
    case CMD_RASET:   return 4;
    case CMD_FRMCTR3: return 6;
    case CMD_GMCTRP1:
    case CMD_GMCTRN1: return 16;
    default:          return 0;
  }
}

void execute_command(chip_state_t *chip) {
  switch (chip->command_code) {
    case CMD_NOP:
      break;

    case CMD_SLPIN:
    case CMD_DISPOFF:
      // Not implemented.
      break;

    case CMD_SLPOUT:
    case CMD_DISPON:
      // Not implemented.
      break;

    case CMD_INVOFF:
    case CMD_INVON:
      // Not implemented.
      break;

    case CMD_RAMWR:
      chip->ram_write = true;
      break;

    case CMD_MADCTL:
      chip->scanning_direction = chip->command_buf[0] & 0xff;
      break;

    case CMD_CASET:
    case CMD_RASET: {
      if (chip->command_size < 4) break;
      uint16_t arg0 = (chip->command_buf[0] << 8) | chip->command_buf[1];
      uint16_t arg2 = (chip->command_buf[2] << 8) | chip->command_buf[3];
      bool set_page = chip->command_code == CMD_RASET;
      if ((chip->scanning_direction & SCAN_MV) ? !set_page : set_page) {
        chip->active_page = arg0;
        chip->page_start = arg0;
        chip->page_end = arg2;
        if (chip->scanning_direction & SCAN_MY) {
          // Some displays use offsets; keep simple and clamp
          if (chip->page_start >= 32) chip->page_start -= 32;
          if (chip->page_end >= 32) chip->page_end -= 32;
          if (chip->active_page >= 32) chip->active_page -= 32;
        }
      } else {
        chip->active_column = arg0;
        chip->column_start = arg0;
        chip->column_end = arg2;
      }
      break;
    }

    case CMD_PWCTR1:
    case CMD_SWRESET:
      chip_reset(chip);
      break;

    case CMD_COLMOD:
    case CMD_VMCTR:
      // Not implemented.
      break;

    default:
      printf("Warning: unknown command 0x%02x\n", chip->command_code);
      break;
  }
}

void process_command(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size) {
  chip->ram_write = false;
  for (uint32_t i = 0; i < buffer_size; i++) {
    chip->command_code = buffer[i];
    chip->command_size = command_args_size(chip->command_code);
    chip->command_index = 0;
    if (!chip->command_size) {
      execute_command(chip);
    }
  }
}

void process_command_args(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size) {
  for (uint32_t i = 0; i < buffer_size; i++) {
    if (chip->command_index < chip->command_size) {
      chip->command_buf[chip->command_index++] = buffer[i];
      if (chip->command_size == chip->command_index) {
        execute_command(chip);
        chip->command_index = 0; // prepare for next command
      }
    }
  }
}

void process_data(chip_state_t *chip, const uint8_t *buf, uint32_t byte_count) {
  // Expecting 16-bit per pixel (RGB565) big-endian: hi, lo
  if (byte_count < 2) return;
  uint32_t pixels = byte_count / 2;
  for (uint32_t i = 0; i < pixels; ++i) {
    uint16_t val = (uint16_t)buf[2*i] << 8 | buf[2*i + 1];

    int x = (int)chip->active_column;
    int y = (int)chip->active_page;
    if (chip->scanning_direction & SCAN_MV) {
      x = (chip->scanning_direction & SCAN_MX) ? (chip->width - 1 - x) : x;
      y = (chip->scanning_direction & SCAN_MY) ? (chip->height - 1 - y) : y;
    } else {
      x = (chip->scanning_direction & SCAN_MY) ? (chip->width - 1 - x) : x;
      y = (chip->scanning_direction & SCAN_MX) ? (chip->height - 1 - y) : y;
    }

    // Clamp coordinates to framebuffer
    if (x < 0 || x >= (int)chip->width || y < 0 || y >= (int)chip->height) {
      // advance pointers according to scanning direction, but skip write
    } else {
      uint32_t color = rgb565_to_rgba(val);
      uint32_t pix_index = (uint32_t)y * chip->width + (uint32_t)x;
      buffer_write(chip->framebuffer, pix_index * sizeof(color), &color, sizeof(color));
    }

    // Advance the write pointer in the configured order
    if (chip->scanning_direction & SCAN_MV) {
      chip->active_page++;
      if (chip->active_page > chip->page_end) {
        chip->active_page = chip->page_start;
        chip->active_column++;
        if (chip->active_column > chip->column_end) {
          chip->active_column = chip->column_start;
        }
      }
    } else {
      chip->active_column++;
      if (chip->active_column > chip->column_end) {
        chip->active_column = chip->column_start;
        chip->active_page++;
        if (chip->active_page > chip->page_end) {
          chip->active_page = chip->page_start;
        }
      }
    }
  }
}

void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t*)user_data;
  if (!count) return; // called from spi_stop probably

  if (chip->mode == MODE_DATA) {
    if (chip->ram_write) {
      // buffer contains raw pixel bytes
      process_data(chip, buffer, count);
    } else {
      // these are arguments for the last command (e.g. CASET/RASET)
      process_command_args(chip, buffer, count);
    }
  } else {
    // command bytes
    process_command(chip, buffer, count);
  }

  if (pin_read(chip->cs_pin) == LOW) {
    // Keep receiving until CS goes high
    spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
  }
}

