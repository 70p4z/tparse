/*******************************************************************************
*   TPARSE Embedded text parsing library
*   (c) 2022 Olivier TOMAZ
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************/

#pragma once

#include "string.h"
#include "stdint.h"

typedef struct tparse_ctx_s {
	uint32_t timeout;
	char* buffer;
	char* delim;
	size_t r_offset;
	size_t w_offset;
	size_t max_length;
	uint32_t flags;
} tparse_ctx_t;
#define TPARSE_EOL        1

#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif // MIN

// to be implemented on the target
uint32_t tparse_al_time(void);

/**
 * Initialize token parsing, delim should include at least \0 and it is advised to
 * also include \r and \n for correct line parsing. Field delimiter is generally ' ' or \t
 * End of line is always \n, no support for \r\n so far
 */
void tparse_init(tparse_ctx_t* ctx, char* buffer, size_t max_length, char* delim);

/**
 * Manual data feeding using polling or anything.
 */
void tparse_append(tparse_ctx_t* ctx, char* chunk, size_t length);

/**
 * Notify of the current write offset by a background process (DMA...).
 * The offset is the current write offset of the DMA within the circular buffer.
 * NOTE: generally, DMA registers holds the remaining data unit transfer count. 
 */
void tparse_finger(tparse_ctx_t* ctx, size_t w_offset);

/**
 * Reset the tparse structure.
 */
void tparse_reset(tparse_ctx_t* ctx);

#define TPARSE_TOKEN_PART (1<<31)
/**
 * Return next token part (if at end of circular buffer)
 * @return TPARSE_TOKEN_PART bit set when only a part of the token is returned
 */
uint32_t tparse_token_p(tparse_ctx_t* ctx, char** token);

/**
 * Copy the next token within the given buffer, if max_size is overflown, then 
 * only the fitting token part is copied but the whole token is consumed. Use 
 * ::tparse_token_size beforehand to provide the correct max_size buffer.
 */
size_t tparse_token(tparse_ctx_t* ctx, char* buf, size_t max_size);

/**
 * Read greedily from the buffer, without consideration for tokenization
 * This API (with ::tparse_avail and ::tparse_finger) enables to use the circular buffer handling of tparse
 */
size_t tparse_read(tparse_ctx_t* ctx, char* buf, size_t max_size);

/** 
 * Return length of unparsed data (not considering token delimiters)
 */ 
size_t tparse_avail(tparse_ctx_t* ctx);

/**
 * Returns the size of the line when a full line is available to parse in the buffer.
 * Returned length includes the EOL char (\n).
 */
size_t tparse_has_line(tparse_ctx_t* ctx);

/**
 * Copy the current received line (return 0 until an end of line is reached)
 */
size_t tparse_peek_line(tparse_ctx_t* ctx, char* buffer, size_t max_length);

/**
 * Returns 1 if a complete token is available (with an end delimiter). Returns 0 when no next token is present
 */
size_t tparse_token_size(tparse_ctx_t* ctx);

/**
 * Return which token from the array matched the next token read from the buffer
 * @return index+1 in the tokens array
 */
uint32_t tparse_token_in(tparse_ctx_t* ctx, char** tokens, size_t count, char* token_consumed, size_t* token_consumed_len_in_out);

/**
 * Read next token as an hexadecimal encoded byte array. 
 */
size_t tparse_token_hex(tparse_ctx_t* ctx, uint8_t* buffer, size_t buffer_length);

/** 
 * Parse the next token as a uint32_t value (supporting decimal and hexadecimal encoding)
 */
uint32_t tparse_token_u32_base(tparse_ctx_t* ctx, uint32_t _base);

/**
 * Read next token as a uint32_t, decimal or hexadecimal encoded
 */
uint32_t tparse_token_u32(tparse_ctx_t* ctx);

/** 
 * Discard everything until end of line, at least a token must have been consumed on the line
 */
void tparse_discard_line(tparse_ctx_t* ctx);

/**
 * Mark the line as consumed, whatever consumed or not
 */
void tparse_consume_line(tparse_ctx_t* ctx);

/**
 * Discard all received data from the uart
 */
void tparse_discard(tparse_ctx_t* ctx);

/**
 * Check if parsing reached EOL
 */
uint32_t tparse_eol_reached(tparse_ctx_t* ctx);

/**
 * Count number of token on the line. When EOL is reached, the count tokens on the next line if any
 * Upon delimiter repetition, it does not account for empty tokens
 */
size_t tparse_token_count(tparse_ctx_t* ctx);
