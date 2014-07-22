# Makefile: build GISSUMO using libpqxx
# Assumes boost libraries installed systemwide and libpqxx at /usr/local/include

CC=g++
CXXFLAGS=-c -O2 -std=c++11 -Wall --pedantic
LDFLAGS=-O2

SOURCES=gissumo.cpp gis.cpp network.cpp uvcast.cpp
EXECUTABLE=rsu-coverage
OBJECTS=$(SOURCES:.cpp=.o)

INCLUDEDIRS=-I/usr/local/include
EXTRALIBS=-lpqxx -lpq -lboost_program_options -lboost_thread


all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) $(EXTRALIBS) $(LDFLAGS) -o $@

.cpp.o:
	$(CC) $< -o $@ $(INCLUDEDIRS) $(CXXFLAGS)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE)
