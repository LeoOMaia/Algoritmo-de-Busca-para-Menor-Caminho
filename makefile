# Nome do arquivo fonte e executável
SRC = pathfinder.cpp
EXEC = pathfinder

# Compilador e flags
CC = g++
CFLAGS = -Wall

# Regra padrão: build
build: $(EXEC)

# Regra para gerar o executável
$(EXEC): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(EXEC)

# Regra para limpar arquivos temporários e o executável
clean:
	rm -f $(EXEC) *.o *~ ./csv_files/*.csv

.PHONY: build clean