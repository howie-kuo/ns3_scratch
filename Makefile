CXX = g++ -std=c++23
CXXFLAGS = -O2
NS3INCLUDE = ../../build/include
NS3LIB = ../../build/lib
BUILD = default

%.o: %.cc
	$(CXX) $(CXXFLAGS) -I$(NS3INCLUDE) -c $< -o $@

lte_ex4: lte_ex4.o
	$(CXX) $(CXXFLAGS)  -L$(NS3LIB) -o lte_ex4 lte_ex4.o \
	-lns3.48-core-$(BUILD) \
	-lns3.48-internet-$(BUILD) \
	-lns3.48-applications-$(BUILD) \
	-lns3.48-point-to-point-$(BUILD) \
	-lns3.48-lte-$(BUILD) \
	-lns3.48-mobility-$(BUILD) \
	-lns3.48-network-$(BUILD)

clean:
	rm -f lte_ex4 lte_ex4.o *.txt

run:
	LD_LIBRARY_PATH='$(LD_LIBRARY_PATH):/home/d887892/src/ns-allinone-3.48/ns-3.48/build/lib' \
	./lte_ex4 --simTime=60 \
	          --activeUes=2 \
			  --ulArrivalRate=0.5 \
			  --dlArrivalRate=0.5