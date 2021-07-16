all:
	g++ -Wall -O3 -o PSO_SA main.cpp

clean:
	rm -f main *.o