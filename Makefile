CC = g++ -std=c++17 
PROJECT = AR
SRC = src/main.cpp src/utils.cpp src/OBJParser.cpp
LIBS = `pkg-config --cflags --libs opencv4`
$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)
