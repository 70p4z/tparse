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

#include "tparse.h"

// to be implemented on the target
__attribute__((weak)) uint32_t tparse_al_time(void) { 
	return 0; 
} 

/**
 * Initialize token parsing, delim should include at least \0 and it is advised to
 * also include \r and \n for correct line parsing. Field delimiter is generally ' ' or \t
 */
void tparse_init(tparse_ctx_t* ctx, char* buffer, size_t max_length, char* delim) {
	memset(ctx, 0, sizeof(tparse_ctx_t));
	memset(buffer, 0, max_length);
	ctx->buffer = buffer;
	ctx->max_length = max_length;
	ctx->delim = delim;
}

/**
 * Manual data feeding usig polling or anything.
 */
void tparse_append(tparse_ctx_t* ctx, char* chunk, size_t length) {
	size_t l;
	// take into account multiple overlays ... user ARE intringuing sometimes
	// TODO when crossing r_offset => move it
	while (length) {
		l = MIN(ctx->max_length - ctx->w_offset, length);
		memmove(ctx->buffer+ctx->w_offset, chunk, l);
		chunk += l;
		length -= l;
		ctx->w_offset = (ctx->w_offset+l)%ctx->max_length;
	}
	ctx->timeout = tparse_al_time();
}


/**
 * Notify of the current write offset by a background process (DMA...).
 * The offset is the current write offset of the DMA within the circular buffer.
 * NOTE: generally, DMA registers holds the remaining data unit transfer count. 
 */
void tparse_finger(tparse_ctx_t* ctx, size_t w_offset) {
	// TODO when crosisng r_offset => move it
	// modulo just in case 8j
	size_t old = ctx->w_offset;
	ctx->w_offset = w_offset % ctx->max_length;
	// reset timeout?
	if (old != ctx->w_offset) {
		ctx->timeout = tparse_al_time();
	}
}

/**
 * Reset the tparse structure.
 */
void tparse_reset(tparse_ctx_t* ctx) {
	memset(ctx->buffer, 0, ctx->max_length);
	ctx->flags = ctx->w_offset = ctx->r_offset = ctx->timeout = 0;
}

/**
 * 
 */
uint32_t tparse_timeout(tparse_ctx_t* ctx) {
	return ctx->timeout && ((tparse_al_time() - ctx->timeout) < 0x8000000UL);
}

void tparse_discard(tparse_ctx_t* ctx) {
	ctx->r_offset = ctx->w_offset;
}

void tparse_discard_line(tparse_ctx_t* ctx) {
	size_t r_offset = ctx->r_offset;
	// already at end of line, and nothing consumed afterwards
	if (ctx->flags & TPARSE_EOL) {
		if (r_offset != ctx->w_offset 
			&& ctx->buffer[r_offset] == '\n') {
			ctx->flags |= TPARSE_EOL;
			// consume the EOL
			ctx->r_offset = (r_offset+1)%ctx->max_length;
		}
		return;
	}
	if (r_offset != ctx->w_offset) {
		do {
			if (ctx->buffer[r_offset] == '\n') {
				ctx->flags |= TPARSE_EOL;
				// consume the EOL char
				ctx->r_offset = (r_offset+1)%ctx->max_length;
				return;
			}
			r_offset = (r_offset+1) %ctx->max_length;
		}
		while (r_offset != ctx->w_offset);
	}
	// if no end of line, then don't consume anything
}

/**
 * Return size of the token, or size of the token first part | (1<<31) when token is on the circular limit.
 * When 1<<31 is set, another call to token shall be issued to retrieve the next token part and the two 
 * parts concat must be done be the caller.
 */
#include "stdio.h"
uint32_t tparse_token_internal(tparse_ctx_t* ctx, char** token, size_t* add_r_offset, uint32_t consume) {
	// parse token up until max_length bytes have been fetched OR \0 is reached
	size_t r_offset = ( ctx->r_offset + *add_r_offset ) % ctx->max_length;
	size_t delim_len = strlen(ctx->delim);
	size_t tlen = 0;
	size_t skipped_delim = 0;
	if (r_offset == ctx->w_offset) {
		return 0;
	}
	*token = &ctx->buffer[r_offset];
	do {
		size_t l = delim_len;
		while(l--) {
			if (ctx->buffer[r_offset] == ctx->delim[l]) {
				// delimiter found, a token ends here
				tlen = (ctx->max_length + r_offset - ctx->r_offset - *add_r_offset) % ctx->max_length - skipped_delim;
				// at least consume some char to create a token, mark delim as consumed
				if (ctx->delim[l] != '\n' && tlen == 0) {
					skipped_delim++;
					*token=&ctx->buffer[r_offset+1]; 
					break;
				}
				if(consume) {
					ctx->r_offset = (r_offset + 1) % ctx->max_length;
					ctx->flags &= ~TPARSE_EOL;
					// if next char is EOL, then account for it
					if (ctx->buffer[r_offset] == '\n') {
						ctx->flags |= TPARSE_EOL;
					}
				}
				*add_r_offset += tlen + skipped_delim;
				return tlen;
			}
		}
		r_offset++;
		// token over the circling limit?
		if (r_offset == ctx->max_length) {
			tlen = r_offset - ctx->r_offset - *add_r_offset - skipped_delim;
			*add_r_offset += tlen + skipped_delim;
			if(consume) {
				ctx->r_offset = 0;
				ctx->flags &= ~TPARSE_EOL;
			}
			return tlen | TPARSE_TOKEN_PART;
		}
	}
	while (r_offset != ctx->r_offset && r_offset != ctx->w_offset && r_offset < ctx->max_length);
	// no token found YET!
	return 0;
}

uint32_t tparse_token_p(tparse_ctx_t* ctx, char** token) {
	size_t s = 0;
	return tparse_token_internal(ctx, token, &s, 1);
}

size_t tparse_avail(tparse_ctx_t* ctx) {
	return ( ctx->max_length + ctx->w_offset - ctx->r_offset ) % ctx->max_length;
}

size_t tparse_read(tparse_ctx_t* ctx, char* buffer, size_t max_length) {
	size_t read = tparse_avail(ctx);
	if (max_length < read) {
		read = max_length;
	}
	// read r to end
	size_t part = ctx->max_length - ctx->r_offset;
	if (read < part) {
		part = read;
	}
	memmove(buffer, &ctx->buffer[ctx->r_offset], part);
	ctx->r_offset = (ctx->r_offset + part) % ctx->max_length;
	buffer += part;
	// read end to w
	if (read - part > 0) {
		part = ctx->w_offset;
		if (read < part) {
			part = read;
		}
		memmove(buffer, &ctx->buffer[ctx->r_offset], part);
		ctx->r_offset = (ctx->r_offset + part) % ctx->max_length;
	}
	return read;
}

size_t tparse_has_line(tparse_ctx_t* ctx) {
	uint32_t avail = tparse_avail(ctx);
	uint32_t r_offset = ctx->r_offset;
	size_t line_length=0;
	while(avail--) {
		line_length++;
		if (ctx->buffer[r_offset] == '\n') {
			return line_length;
		}
		r_offset = (r_offset + 1) % ctx->max_length;
	}
	return 0;
}

size_t tparse_peek_line(tparse_ctx_t* ctx, char* buffer, size_t max_length) {
	uint32_t avail = tparse_avail(ctx);
	uint32_t r_offset = ctx->r_offset;
	size_t line_length=0;
	while(avail--) {
		char c = ctx->buffer[r_offset];
		if (line_length<max_length) {
			*buffer = c;
			buffer++;
		}
		line_length++;
		if (c == '\n') {
			if (line_length<max_length) {
				*buffer = '\0'; // EOL if still space
				line_length++;
			}
			return line_length;
		}
		r_offset = (r_offset + 1) % ctx->max_length;
	}
	return 0;
}

uint32_t tparse_eol_reached(tparse_ctx_t* ctx) {
	return ctx->flags & TPARSE_EOL;
}

size_t tparse_token_count(tparse_ctx_t* ctx) {
	// count tokens until
	uint32_t off=0;
	uint32_t len=tparse_avail(ctx);
	size_t count=0;
	char* token;
	// when eol, then count tokens o the following line
	if (len == 0) {
		return 0;
	}
	while (off<len) {
		size_t l = tparse_token_internal(ctx, &token, &off, 0); 
		if (l==0) {
			off++;
			continue;
		}
		//off+=l&~TPARSE_TOKEN_PART;
		if (l&TPARSE_TOKEN_PART) {
			l = tparse_token_internal(ctx, &token, &off, 0); 
		}
		count++;
		// TEST when count next line is on the rollover
		if (ctx->buffer[off%ctx->max_length] == '\n') {
			break;
		}
		off++; // account for the delimiter
	}
	return count;
}


/**
 * Returns 1 if a complete token is available (with an end delimiter)
 */
size_t tparse_token_size(tparse_ctx_t* ctx) {
	char* t;
	size_t s = 0;
	size_t l = tparse_token_internal(ctx, &t, &s, 0);
	size_t l2;
	if (l) {
		// note: l can have the EOL set
		if (!(l&TPARSE_TOKEN_PART)) {
			return l;
		}
		// parse remaining part (if any)
		s = l&~TPARSE_TOKEN_PART;
		l2 = tparse_token_internal(ctx, &t, &s, 0);
		// unsupported: this is clearly an overflow of the buffer. enjoy
		if (l2&TPARSE_TOKEN_PART) {
			return 0;
		}
		// note: l2 can have the EOL set
		return (l&~TPARSE_TOKEN_PART) + l2;
	}
	return 0;
}

size_t tparse_token_parts(tparse_ctx_t* ctx, char** tokens, size_t* tokens_size) {
	if (!tparse_token_size(ctx)) {
		return 0;
	}
	tokens_size[0] = tparse_token_p(ctx, &tokens[0]);
	tokens[1] = NULL;
	tokens_size[1] = 0;
	if (tokens_size[0] & TPARSE_TOKEN_PART) {
		tokens_size[0] &= ~TPARSE_TOKEN_PART;
		// if next delim starts with delim, then part0 ended.
		size_t l = strlen(ctx->delim);
		while(l--) {
			if (ctx->buffer[0] == ctx->delim[l]) {
				return tokens_size[0];
			}
		}
		tokens_size[1] = tparse_token_p(ctx, &tokens[1]);
	}
	return tokens_size[0] + tokens_size[1];
}

size_t tparse_token(tparse_ctx_t* ctx, char* buf, size_t len) {
	char* parts[2]; // when token is split due to circular mode
	size_t sizes[2];
	size_t l = tparse_token_parts(ctx, parts, sizes);
	// nothing to be consumed
	if (!l) {
		return 0;
	}
	l = MIN(l, len);
	len = l;
	while (l && sizes[0]) {
		l--;
		sizes[0]--;
		*buf++ = *parts[0]++;
	}
	while (l && sizes[1]) {
		l--;
		sizes[1]--;
		*buf++ = *parts[1]++;
	}
	return len;
}

/**
 * Return which token from the array matched the next token read from the buffer
 */
uint32_t tparse_token_in(tparse_ctx_t* ctx, char** tokens, size_t count, char* token_consumed, size_t* token_consumed_len_in_out) {
	char* parts[2]; // when token is split due to circular mode
	size_t sizes[2];
	size_t entry=0;

	if (token_consumed_len_in_out) {
		*token_consumed_len_in_out=0;
	}
	if (!tparse_token_parts(ctx, parts, sizes)) {
		return 0;
	}
	while(count--) {
		if (memcmp(tokens[count], parts[0], sizes[0]&~TPARSE_TOKEN_PART) == 0) {
			if (strlen(tokens[count]) == sizes[0]) {
				entry=count+1;
				goto copy_consumed_token;
			}
			if (sizes[1] && memcmp(tokens[count]+sizes[0], parts[1], sizes[1]) == 0) {
				if (strlen(tokens[count]) == sizes[0] + sizes[1]) {
					entry=count+1;
					goto copy_consumed_token;
				}
			}
		}
	}
copy_consumed_token:
	// if no match, then the first token is read and optionally returned.
	if (token_consumed) {
		count = *token_consumed_len_in_out = MIN(sizes[0] + sizes[1], *token_consumed_len_in_out);
		while (count && sizes[0]) {
			count--;
			sizes[0]--;
			*token_consumed++ = *parts[0]++;
		}
		while (count && sizes[1]) {
			count--;
			sizes[1]--;
			*token_consumed++ = *parts[1]++;
		}
	}
	return entry;
}

/**
 * Read next token as an hexadecimal encoded byte array. 
 */
size_t tparse_token_hex(tparse_ctx_t* ctx, uint8_t* buffer, size_t buffer_length) {
	char* parts[2]; // when token is split due to circular mode
	size_t sizes[2];
	size_t l = tparse_token_parts(ctx, parts, sizes);
	size_t len=0;
	if (!l) {
		return 0;
	}

	// requires a bigger byte array to store content
	if (l/2 + (l%2) > buffer_length) {
		return 0;
	}
	uint8_t c;
	uint8_t i=0;
	while (l-- && buffer_length) {
		if (i==0) {
			len++;
			*buffer=0;
		}
		*buffer<<=4;
		// select source token part
		if (sizes[0]) {
			sizes[0]--;
			c=*parts[0]++; 
		}
		else if (sizes[1]) {
			sizes[1]--;
			c=*parts[1]++;
		}
		else {
			// End Of Token
			return 0;
		}
		// convert
		if (c >= '0' && c <= '9') {
			c = c - '0';
		}
		else if (c >= 'a' && c <= 'f') {
			c = c - 'a' + 10;
		}
		else if (c >= 'A' && c <= 'F') {
			c = c - 'A' + 10;
		}
		else {
			// non hex char encountered
			return 0;
		}
		*buffer |= c; // &9xF is done by construction
		// prepare next dest
		i++;
		if(i==2) {
			i=0;
			buffer++;
			buffer_length--;
		}
	}
	return len;
}

/** 
 * Parse the next token as a uint32_t value (supporting decimal and hexadecimal encoding)
 */
uint32_t tparse_token_u32_base(tparse_ctx_t* ctx, uint32_t _base) {
	char* parts[2]; // when token is split due to circular mode
	size_t sizes[2];
	size_t l = tparse_token_parts(ctx, parts, sizes);
	size_t len=0;
	if (!l) {
		return -1;
	}

	uint32_t value=0;
	uint8_t base=_base;
	uint8_t c;
	uint32_t neg=0;
	while (l--) {
		// select source token part
		if (sizes[0]) {
			sizes[0]--;
			c=*parts[0]++; 
		}
		else if (sizes[1]) {
			sizes[1]--;
			c=*parts[1]++;
		}
		else {
			// End Of Token
			return -1; // robustness
		}
		// starts with 0x...
		if (value == 0 && len == 1 && c == 'x') {
			base=16;
		}
		// detect signed numbers
		else if (len==0 && c=='-') {
			// forcefully the first char!
			neg=1;
		}
		else {
			switch(base) {
				case 10:
					if (c >= '0' && c <= '9') {
						c = c - '0'; 
					}
					else {
						return -1;
					}
					break;
				case 16:
					// convert
					if (c >= '0' && c <= '9') {
						c = c - '0';
					}
					else if (c >= 'a' && c <= 'f') {
						c = c - 'a' + 10;
					}
					else if (c >= 'A' && c <= 'F') {
						c = c - 'A' + 10;
					}
					else {
						// non hex char encountered
						return -1;
					}
					break;
			}
			value = value * base + c;
		}
		len++;
	}
	if (neg) {
		value = (uint32_t)-value;
	}
	return value;
}

/**
 * Default base is 10
 */
uint32_t tparse_token_u32(tparse_ctx_t* ctx) {
	return tparse_token_u32_base(ctx, 10);
}

int32_t tparse_token_i32(tparse_ctx_t* ctx) {
	return (int32_t)tparse_token_u32_base(ctx, 10);
}
