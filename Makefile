CPP = g++
CPPFLAGS = -Wall -pedantic -O3 -flto -march=native -I /usr/local/include
BENCHMARK = -D RUN_BENCHMARK
BENCH_EXE = ray_bench
TESTFLAGS = -Wall -pedantic -g
SOURCES = trace.cpp world.cpp
DEP = ray.h Makefile
OBJECTS=$(SOURCES:.cpp=.o)
TESTEXES=$(SOURCES:.cpp=)
EXE=ray
IMAGE=out.png

all: $(EXE)

$(EXE): $(OBJECTS) $(DEP)
	$(CPP) $(CPPFLAGS) $(OBJECTS) -o $@

%.o: %.cpp $(DEP)
	$(CPP) $(CPPFLAGS) -c $< -o $@

%: %.cpp $(DEP)
	$(CPP) $(TESTFLAGS) $< -o $@

clean:
	-rm -f $(TESTEXES) *.o $(EXE)

run: all
	./$(EXE)

$(BENCH_EXE): $(SOURCES) $(DEP)
	$(CPP) $(CPPFLAGS) $(BENCHMARK) $(SOURCES) -o $@

bench: $(BENCH_EXE)
	./$(BENCH_EXE) 1>/dev/null 2>log.txt

$(IMAGE): all
	./$(EXE) | pnmtopng > $(IMAGE)

fullscreen: $(IMAGE)
	eog -f $(IMAGE)

disp: $(IMAGE)
	eog $(IMAGE)

runtrace: trace
	./trace

runworld: world
	./world
