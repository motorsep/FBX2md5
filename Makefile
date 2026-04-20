# ----------------------------------------------------------------------
#  Makefile for fbx2md5 (Linux / macOS).
#
#  For Windows, use generate_vs2022.bat instead. This Makefile is a
#  stand-alone alternative to CMakeLists.txt -- no CMake is required.
#
#  Targets:
#      make          build Release (default)
#      make debug    build with -O0 -g
#      make clean    remove the build/ directory
# ----------------------------------------------------------------------

CXX      ?= g++
CC       ?= gcc
CXXFLAGS ?= -O2 -std=c++17 -Wall
CFLAGS   ?= -O2 -Wall
LDFLAGS  ?=
LDLIBS   ?= -lm -lpthread

SRCDIR   := src
BUILDDIR := build
TARGET   := $(BUILDDIR)/fbx2md5

CXX_OBJS := $(BUILDDIR)/fbx2md5.o
C_OBJS   := $(BUILDDIR)/ufbx.o
OBJS     := $(CXX_OBJS) $(C_OBJS)

.PHONY: all debug clean
all: $(TARGET)

debug: CXXFLAGS := -O0 -g -std=c++17 -Wall
debug: CFLAGS   := -O0 -g -Wall
debug: $(TARGET)

$(TARGET): $(OBJS) | $(BUILDDIR)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BUILDDIR)/fbx2md5.o: $(SRCDIR)/fbx2md5.cpp $(SRCDIR)/ufbx.h | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -I$(SRCDIR) -c $< -o $@

$(BUILDDIR)/ufbx.o: $(SRCDIR)/ufbx.c $(SRCDIR)/ufbx.h | $(BUILDDIR)
	$(CC) $(CFLAGS) -I$(SRCDIR) -c $< -o $@

$(BUILDDIR):
	@mkdir -p $(BUILDDIR)

clean:
	rm -rf $(BUILDDIR)
