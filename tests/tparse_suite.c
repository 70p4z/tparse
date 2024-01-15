/*******************************************************************************
*   TPARSE test suite
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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <munit.h>
#include <tparse.h>

#define STR_AND_LEN(x) x, sizeof(x) -1

uint8_t buffer[2048];

static MunitResult* test_basic(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 0);
  

  tparse_append(&ctx, STR_AND_LEN("super truc\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 11);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 2);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "super", t);

  munit_assert_int(tparse_avail(&ctx), ==, 5);
  munit_assert_int(tparse_token_size(&ctx), ==, 4);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 4);
  munit_assert_memory_equal(4, "truc", t);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 1);
  munit_assert_int(tparse_token_count(&ctx), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("autre machin\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 13);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  //munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 2);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "autre", t);

  munit_assert_int(tparse_avail(&ctx), ==, 7);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 6);
  munit_assert_memory_equal(6, "machin", t);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 1);
  munit_assert_int(tparse_token_count(&ctx), ==, 0);

  //                                      v last place of the buffer
  tparse_append(&ctx, STR_AND_LEN("bidule chouette\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 16);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  //munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 2);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 6);
  munit_assert_memory_equal(6, "bidule", t);

  munit_assert_int(tparse_avail(&ctx), ==, 9);
  munit_assert_int(tparse_token_size(&ctx), ==, 8);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0x80000001);
  munit_assert_memory_equal(1, "c", t);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 0);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 7);
  munit_assert_memory_equal(7, "houette", t);
  munit_assert_int(tparse_eol_reached(&ctx), ==, 1);
  munit_assert_int(tparse_token_count(&ctx), ==, 0);

  return MUNIT_OK;
}

static MunitResult* test_hex(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  uint8_t buf[128];

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("val 43BEd2e6\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 13);
  munit_assert_int(tparse_token_size(&ctx), ==, 3);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 3);
  munit_assert_memory_equal(3, "val", t);

  munit_assert_int(tparse_avail(&ctx), ==, 9);
  munit_assert_int(tparse_token_size(&ctx), ==, 8);
  munit_assert_int(tparse_token_hex(&ctx, buf, 128), ==, 4);
  munit_assert_memory_equal(4, "\x43\xBE\xd2\xe6", buf);

  tparse_append(&ctx, STR_AND_LEN("12fe9bc4a2870ecB6d282109\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 25);
  munit_assert_int(tparse_token_size(&ctx), ==, 24);
  munit_assert_int(tparse_token_hex(&ctx, buf, 128), ==, 12);
  munit_assert_memory_equal(12, "\x12\xfe\x9b\xc4\xa2\x87\x0e\xcB\x6d\x28\x21\x09", buf);

  return MUNIT_OK;
}

static MunitResult* test_in(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  uint8_t buf[128];

  const char* const cmds[] = {
    "cmd1",
    "cmd2",
    "cmd3",
    "cmd4",
  };

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("cmd cmd1 len cnn2\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 18);
  munit_assert_int(tparse_token_size(&ctx), ==, 3);
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 0);
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 1);

  tparse_append(&ctx, STR_AND_LEN("super token cmd4 cmd2\n"));
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 0); // len
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 0); // cnn2
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 0); // super
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 0); // token
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 4); // over the circular rotation
  munit_assert_int(tparse_token_in(&ctx, cmds, 4, NULL, 0), ==, 2); // after rotation

  return MUNIT_OK;
}

static MunitResult* test_u32(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  uint8_t buf[128];

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("12 2567\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 12);
  munit_assert_int(tparse_token_u32(&ctx), ==, 2567);

  //                                                     v circular limit
  tparse_append(&ctx, STR_AND_LEN("13 2568 3546 324264 1456644322\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 13);
  munit_assert_int(tparse_token_u32(&ctx), ==, 2568);
  munit_assert_int(tparse_token_u32(&ctx), ==, 3546);
  munit_assert_int(tparse_token_u32(&ctx), ==, 324264);
  munit_assert_int(tparse_token_u32(&ctx), ==, 1456644322);

  tparse_append(&ctx, STR_AND_LEN("11 0x2576\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 11);
  munit_assert_int(tparse_token_u32(&ctx), ==, 0x2576);

  tparse_append(&ctx, STR_AND_LEN("11 0 1\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 11);
  munit_assert_int(tparse_token_u32(&ctx), ==, 0);
  munit_assert_int(tparse_token_u32(&ctx), ==, 1);

  return MUNIT_OK;
}

static MunitResult* test_s32(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  uint8_t buf[128];

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("12 -2567\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 12);
  munit_assert_int(tparse_token_u32(&ctx), ==, -2567);

  //                                                     v circular limit
  tparse_append(&ctx, STR_AND_LEN("13 256 3546 -324264 1456644322\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 13);
  munit_assert_int(tparse_token_u32(&ctx), ==, 256);
  munit_assert_int(tparse_token_u32(&ctx), ==, 3546);
  munit_assert_int(tparse_token_u32(&ctx), ==, -324264);
  munit_assert_int(tparse_token_u32(&ctx), ==, 1456644322);

  tparse_append(&ctx, STR_AND_LEN("11 0x2576\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 11);
  munit_assert_int(tparse_token_u32(&ctx), ==, 0x2576);

  tparse_append(&ctx, STR_AND_LEN("11 0 -1\n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 11);
  munit_assert_int(tparse_token_u32(&ctx), ==, 0);
  munit_assert_int(tparse_token_u32(&ctx), ==, -1);

  tparse_append(&ctx, STR_AND_LEN("394687 -5704 31000  \n"));
  munit_assert_int(tparse_token_u32(&ctx), ==, 394687);
  munit_assert_int(tparse_token_i32(&ctx), ==, -5704);
  munit_assert_int(tparse_token_u32(&ctx), ==, 31000);

  return MUNIT_OK;
}

static MunitResult* test_eol(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  uint8_t buf[128];

  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("super token\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 12);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "super", t);
  munit_assert_int(tparse_avail(&ctx), ==, 6);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "token", t);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  memset(buffer, 0, sizeof(buffer));
  tparse_reset(&ctx);
  tparse_append(&ctx, STR_AND_LEN("super token\n"));
  munit_assert_int(tparse_avail(&ctx), ==, 12);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "super", t);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  memset(buffer, 0, sizeof(buffer));
  tparse_reset(&ctx);
  tparse_append(&ctx, STR_AND_LEN("super token\n"));
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_has_line(&ctx), ==, 0);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  //                                                  v cycling
  tparse_append(&ctx, STR_AND_LEN("dupper stem\nhype totem\nola\n"));
  munit_assert_int(tparse_has_line(&ctx), ==, 12);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 6);
  munit_assert_memory_equal(6, "dupper", t);
  munit_assert_int(tparse_token_size(&ctx), ==, 4);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 4);
  munit_assert_memory_equal(4, "stem", t);
  munit_assert_int(tparse_token_size(&ctx), ==, 4);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 4);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_has_line(&ctx), ==, 4);
  munit_assert_int(tparse_avail(&ctx), ==, 4);
  munit_assert_int(tparse_token_size(&ctx), ==, 3);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 3);
  munit_assert_memory_equal(3, "ola", t);

  return MUNIT_OK;
}

static MunitResult* test_eoleol(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  uint8_t buf[128];

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_has_line(&ctx), ==, 0);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("super token\n\ntoken2\n"));
  munit_assert_int(tparse_has_line(&ctx), ==, 12);
  munit_assert_int(tparse_avail(&ctx), ==, 20);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "super", t);
  munit_assert_int(tparse_avail(&ctx), ==, 14);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 5);
  munit_assert_memory_equal(5, "token", t);
  munit_assert_int(tparse_avail(&ctx), ==, 8);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_has_line(&ctx), ==, 7);
  munit_assert_int(tparse_avail(&ctx), ==, 7);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 6);
  munit_assert_memory_equal(6, "token2", t);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_has_line(&ctx), ==, 0);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_reset(&ctx);
  tparse_append(&ctx, STR_AND_LEN("super token\n\ntoken2\n"));
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_has_line(&ctx), ==, 1); // current line is empty
  munit_assert_int(tparse_avail(&ctx), ==, 8); // \ntoken2\n
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_has_line(&ctx), ==, 7);
  munit_assert_int(tparse_avail(&ctx), ==, 7);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 6);
  munit_assert_memory_equal(6, "token2", t);
  munit_assert_int(tparse_has_line(&ctx), ==, 0);
}

static MunitResult* test_discard_no_eol(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  munit_assert_int(tparse_token_size(&ctx), ==, 0);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 0);

  tparse_append(&ctx, STR_AND_LEN("super token"));
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 11);
  munit_assert_int(tparse_token_size(&ctx), ==, 5);

  tparse_reset(&ctx);
  tparse_append(&ctx, STR_AND_LEN("super token\n"));
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
  
  tparse_reset(&ctx);
  tparse_append(&ctx, STR_AND_LEN("super token\n\n"));
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 1);
  tparse_discard_line(&ctx);
  munit_assert_int(tparse_avail(&ctx), ==, 0);
}

static MunitResult* test_copy(const MunitParameter* params, void* ignored) {
  tparse_ctx_t ctx;
  char* t;
  char buf[128];

  memset(buffer, 0, sizeof(buffer));
  tparse_init(&ctx, buffer, 32, " \n\r");

  tparse_append(&ctx, STR_AND_LEN("super truc\n"));
  munit_assert_int(tparse_token(&ctx, buf, 128), ==, 5);
  munit_assert_memory_equal(5, "super", buf);
  munit_assert_int(tparse_token(&ctx, buf, 128), ==, 4);
  munit_assert_memory_equal(4, "truc", buf);

  tparse_append(&ctx, STR_AND_LEN("autre machin\n"));
  munit_assert_int(tparse_token(&ctx, buf, 128), ==, 5);
  munit_assert_memory_equal(5, "autre", buf);
  munit_assert_int(tparse_token(&ctx, buf, 128), ==, 6);
  munit_assert_memory_equal(6, "machin", buf);

  //                                      v last place of the buffer
  tparse_append(&ctx, STR_AND_LEN("bidule chouette\n"));
  munit_assert_int(tparse_token(&ctx, buf, 128), ==, 6);
  munit_assert_memory_equal(6, "bidule", buf);
  munit_assert_int(tparse_token(&ctx, buf, 128), ==, 8);
  munit_assert_memory_equal(8, "chouette", buf);

  return MUNIT_OK;
}

static MunitResult* test_non_reg_token_hex_split(const MunitParameter* params, void* ignored) {
  char * t;
  tparse_ctx_t ctx = {
    .timeout = 0xea38, 
    .buffer = "36ca9d2e079a97aefbd7722a79d5d3af0da7f89fa4b186433c439b5c1a42dd931ec69ff325e1b90a0ac3dd2307955ad710c698ed94fe6e61b7622658506bc2b91dfba4304a8d8f7855fc65074046e4a16de0cf4ddd9787aa92056102927a9a8537567652b9a769b996e1d41f3146495a4d34942b18b80fcc45fdeed9020347bd75aceb42719f401c4f5f2017686960ffaa5f19117c3675cd733f1c57fdb41344897335e5007b02369a60cd5c488b2153372\n0000080e4e3d114afbfc3\nt0 84320000fe0001002004003827140a6655a67ee6a25472365d3d8cd7aaa56b211f88a8873ff41ecd48c5dce076bdb0097438b57e0cfcf961b3418f41c192951f82a1069b273745ded22e406771c270169fa\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", 
    .delim = " \n", 
    .r_offset = 0x17a, 
    .w_offset = 0x164, 
    .max_length = 0x220, 
    .flags = 0x0
  };
  uint8_t tmp[300];
  munit_assert_int(tparse_has_line(&ctx), ==, 522);
  munit_assert_int(tparse_token_count(&ctx), ==, 2);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 2);
  munit_assert_memory_equal(2, t, "t0");
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_size(&ctx), ==, (5+0xFE)*2);
  munit_assert_int(tparse_token_hex(&ctx, tmp, sizeof(tmp)), ==, (5+0xFE));
  return MUNIT_OK;
}

static MunitResult* test_non_reg_nl_after_rollover(const MunitParameter* params, void* ignored) {
  char * t;
  tparse_ctx_t ctx = {
    .timeout = 0xbaf, 
    .buffer = "\nvail\ncavail\ncavail\ncavail\ncavail", 
    .delim = " \n", 
    .r_offset = 27, 
    .w_offset = 1, 
    .max_length = 33, 
    .flags = 0x1
  };
  uint8_t tmp[300];
  munit_assert_int(tparse_has_line(&ctx), ==, 7);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  munit_assert_int(tparse_token_p(&ctx, &t), ==, 6|TPARSE_TOKEN_PART);
  munit_assert_memory_equal(6, t, "cavail");
  return MUNIT_OK;
}

static MunitResult* test_non_reg_nl_after_token_copy(const MunitParameter* params, void* ignored) {
  char * t;
  tparse_ctx_t ctx = {
    .timeout = 0xbaf, 
    .buffer = "\nvail\ncavail\ncavail\ncavail\ncavail", 
    .delim = " \n", 
    .r_offset = 27, 
    .w_offset = 1, 
    .max_length = 33, 
    .flags = 0x1
  };
  uint8_t tmp[300];
  munit_assert_int(tparse_has_line(&ctx), ==, 7);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  munit_assert_int(tparse_token(&ctx, tmp, sizeof(tmp)), ==, 6);
  munit_assert_memory_equal(6, tmp, "cavail");
  return MUNIT_OK;
}

static MunitResult* test_non_reg_nl_after_rollover_token_in(const MunitParameter* params, void* ignored) {
  char * t;
  tparse_ctx_t ctx = {
    .timeout = 0xbaf, 
    .buffer = "\nvail\ncavail\ncavail\ncavail\ncavail", 
    .delim = " \n", 
    .r_offset = 27, 
    .w_offset = 1, 
    .max_length = 33, 
    .flags = 0x1
  };
  uint8_t tmp[300];
  munit_assert_int(tparse_has_line(&ctx), ==, 7);
  munit_assert_int(tparse_token_count(&ctx), ==, 1);
  munit_assert_int(tparse_token_size(&ctx), ==, 6);
  static const char* const cmds[] = {
    "cavail",
  };
  size_t tmp_size=sizeof(tmp);
  munit_assert_int(tparse_token_in(&ctx, cmds, 1, tmp, &tmp_size), ==, 1);
  return MUNIT_OK;
}

///////////////////////////////////////////////////////////////////////////
// TEST LISTS

static MunitTest tests_tparse_suite[] = {
  { /* name */ (char*) "/basic",  test_basic, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/hex",  test_hex, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/in",  test_in, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/u32",  test_u32, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/s32",  test_s32, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/eol",  test_eol, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/eoleol",  test_eoleol, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/discard_no_eol",  test_discard_no_eol, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/copy",  test_copy, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },

  { /* name */ (char*) "/nonreg1",  test_non_reg_token_hex_split, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/nonreg2",  test_non_reg_nl_after_rollover, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/nonreg3",  test_non_reg_nl_after_token_copy, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  { /* name */ (char*) "/nonreg4",  test_non_reg_nl_after_rollover_token_in, /* setup */ NULL, /* tear_down */ NULL, MUNIT_TEST_OPTION_NONE, NULL },
  
  
  { NULL, NULL, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL }
};

// TODO udp checksum (odd and even packet length)

///////////////////////////////////////////////////////////////////////////
// TEST SUITES

const MunitSuite tparse_suite = {
  (char*) "/tparse",
  tests_tparse_suite,
  NULL,
  1,
  /* Just like MUNIT_TEST_OPTION_NONE, you can provide
   * MUNIT_SUITE_OPTION_NONE or 0 to use the default settings. */
  MUNIT_SUITE_OPTION_NONE
};


