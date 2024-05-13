# Definindo o compilador e seu caminho
CC := /home/marcelo/Desktop/IMU/CrossCompile/bin/arm-none-linux-gnueabihf-gcc

# Opções de compilação
CFLAGS := -Wall -Wextra
LDFLAGS += -L/path/to/math/library
LDFLAGS += -lm -pthread
# Diretório contendo o código-fonte
SRC_DIR := /home/marcelo/Desktop/IMU/Coding/ASM330LHB_Driver

# Lista de todos os arquivos .c na pasta de origem
SRCS := $(wildcard $(SRC_DIR)/*.c) $(wildcard $(SRC_DIR)/ASM330LHB/*.c) $(wildcard $(SRC_DIR)/kalman/*.c)

# Nome do executável final
TARGET := IMU

# Endereço de destino para copiar o executável
REMOTE_TARGET := root@10.42.0.133:/IMU

# Comando para limpar os arquivos compilados
RM := rm -rf

# Comando de compilação
$(TARGET): $(SRCS:.c=.o)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	$(RM) $(SRCS:.c=.o) 
	scp $@ $(REMOTE_TARGET)

# Regra para limpar os arquivos compilados
clean:
	$(RM) $(TARGET)

# Indicando que 'clean' não é o nome de um arquivo
.PHONY: clean
