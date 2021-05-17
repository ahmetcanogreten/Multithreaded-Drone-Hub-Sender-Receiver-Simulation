

all:
	gcc simulation.c writeOutput.c helper.c -o simulator -pthread

debug:
	gcc -g simulation.c writeOutput.c helper.c -o debug -pthread

clear:
	rm -rf simulator