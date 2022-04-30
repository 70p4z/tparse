#/*******************************************************************************
#*   TPARSE test suite
#*   (c) 2022 Olivier TOMAZ
#*
#*  Licensed under the Apache License, Version 2.0 (the "License");
#*  you may not use this file except in compliance with the License.
#*  You may obtain a copy of the License at
#*
#*      http://www.apache.org/licenses/LICENSE-2.0
#*
#*  Unless required by applicable law or agreed to in writing, software
#*  distributed under the License is distributed on an "AS IS" BASIS,
#*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#*  See the License for the specific language governing permissions and
#*  limitations under the License.
#********************************************************************************/

############
# Platform #
############

MUNIT := munit/

##############
# Compilator #
##############
CC       := gcc
CFLAGS   := -std=c99 -O0 -ggdb3 -fstack-protector-all --param ssp-buffer-size=1 -Wstack-protector -Wall -I$(MUNIT)
#CFLAGS  += -fpack-struct
#CLFAGS  := -O3
LD       := ld
LDFLAGS  := -O3 -ggdb3 -fstack-protector-all -Wstack-protector -Wl,--no-relax

DEFINES  +=

#########################################################################
# END-OF-CONFIG END-OF-CONFIG END-OF-CONFIG END-OF-CONFIG END-OF-CONFIG #
#########################################################################

#Global include path for all module 'm'
INCLUDES_PATH := . munit/ ../include/
SOURCE_PATH   := . munit/ ../src/

# common munit framework
SOURCE_FILES := $(foreach d,$(SOURCE_PATH),$(shell find $(d) -name "*.[c]"))
OBJECT_FILES := $(addprefix obj/, $(addsuffix  .o, $(basename $(notdir $(SOURCE_FILES)))))

$(info $(OBJECT_FILES))

VPATH := . $(dir $(SOURCE_FILES))
.SECONDARY:

all: default

prepare:
	mkdir -p obj bin
clean:
	rm -rf obj bin

#WARN: leave this include here
ifneq ($(MAKECMDGOALS), clean)
include $(DEPEND_FILES)
endif

#default:  init $(LIB_FILES)
default:  | bin/exe

bin/exe: $(OBJECT_FILES) #$(LIB_FILES)
	$(CC) $(LDFLAGS) $(CFLAGS) obj/*.o -o $@

.SECONDEXPANSION:
#object production
obj/%.o: %.c Makefile prepare
	$(CC) -c $(CFLAGS) $(addprefix -D,$(DEFINES)) $(addprefix -I, $(INCLUDES_PATH)) -o $@  $< 
