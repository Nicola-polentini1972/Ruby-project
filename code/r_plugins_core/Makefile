LDFLAGS=-lrt -lpcap -lpthread
CPPFLAGS=-Wall -O0 -D _GNU_SOURCE
CFLAGS := $(CFLAGS)
RELEASE_DIR = ../../
CC ?= gcc
CXX ?= g++

all: plugin_core_example ruby_core_plugin_histogram_rgb

core_plugins_utils.o: ../public/utils/core_plugins_utils.c
	$(CC) -c -o $@ $< $(CPPFLAGS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -c -o $@ $< $(CPPFLAGS)


plugin_core_example: core_plugins_utils.o plugin_core_example.o
	$(CC) plugin_core_example.o core_plugins_utils.o -shared -Wl,-soname,plugin_core_example.so.1 -o plugin_core_example.so.1.0.1 -lc
	cp -f plugin_core_example.so.1.0.1 $(RELEASE_DIR)
	$(info Copy plugin_core_example)
	$(info ----------------------------------------------------)

ruby_core_plugin_histogram_rgb: core_plugins_utils.o ruby_core_plugin_histogram_rgb.o
	$(CC) ruby_core_plugin_histogram_rgb.o core_plugins_utils.o -shared -Wl,-soname,ruby_core_plugin_histogram_rgb.so.1 -o ruby_core_plugin_histogram_rgb.so.1.0.1 -lc -lpthread
	cp -f ruby_core_plugin_histogram_rgb.so.1.0.1 $(RELEASE_DIR)
	$(info Copy ruby_core_plugin_histogram_rgb)
	$(info ----------------------------------------------------)

clean:
	rm -f plugin_core_example.so.* ruby_core_plugin_histogram_rgb.so.* *.o
