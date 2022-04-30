#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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

#include <munit.h>

// could add some inception mode thanks to some makefile support. would be greatly appreciated
extern const MunitSuite tparse_suite;

int main(int argc, char** argv) {
  int error=0;

  error |= munit_suite_main(&tparse_suite, (void*) "/tparse", argc, argv);

  return error;
}
