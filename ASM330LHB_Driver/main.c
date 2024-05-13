#include <pthread.h>
#include <stdio.h>
#include <unistd.h> // Para sleep()

#include "ASM330LHB/ASM330LHB.h"
//#include "kalman/kalman.h"
#define STORAGE_PATH    "/IMU/BlackBox.txt"
#define SAMPLE_TIME     0.00518f     //This time is the real time mesured between samples in seconds (5.18ms = 66bytes tranfered)
#define SIZE_TEST       127413       //Bytes ->Corresponds to 10 seconds of data (Replace by size_test and verify the file clean at 10 seconds)
#define MAX_SIZE        16000000     //Bytes -> 16MB of space corresponding to 21min of IMU data
#define NUM_THREADS     3

static __u8 ASM330LHB_uartTransmit(ASM330LHB *imu, char* message);
static __u8 ASM330LHB_uartInit(ASM330LHB *imu);

//static __u8 ASM330LHB_uartInit(ASM330LHB *imu);
//static __u8 ASM330LHB_uartTransmit(ASM330LHB *imu, char* message);

typedef enum{
    REAL_TIME,
    REPLAY     //no replay o calculo do roll e do pitch tem de parar
   // OFF
}operatingMode;

typedef struct{
    operatingMode mode;
    ASM330LHB imu;
    pthread_mutex_t mutex;
} IMUData;

/**
 * @brief Task processing the IMU behaviour
*/
void *readIMU(void *arg){
    IMUData *RTOS = (IMUData *)arg;
    
    ASM330LHB_Init(&(RTOS->imu), GYRO_ODR_1670_HZ, ACC_ODR_1670_HZ, GYRO_RANGE_500_DPS, ACC_RANGE_2_G);

    while(1){
        if(RTOS->mode == REPLAY)    //In replay mode there is no need to process imu data
            break; 
        pthread_mutex_lock(&(RTOS->mutex));
        ASM330LHB_Process(&(RTOS->imu));
        pthread_mutex_unlock(&(RTOS->mutex));

        usleep(5000);
    }
    return NULL;
}

/**
 * @brief Task that implements Circular Logging by replacing the oldest data with recent one
 * keeping the text file with the most recent 24 hours IMU data.
*/
void *manageStorage(void *arg){
    IMUData *RTOS = (IMUData *)arg;

    char temp[MAX_SIZE / 2];                    // Temporary Buffer to save the twelve most recent hours of data
    FILE *file = fopen(STORAGE_PATH, "a+");     //  Opens in Read and Write mode
    if (file == NULL) {
        perror("Error opening file.\n");
        return NULL;
    }

    while(1)
    {
        if(RTOS->mode == REPLAY)                            //In replay mode there is no need to manageStorage
            break;        
        fseek(file, 0, SEEK_END);                           //  Places the file pointer in the end to check the file size
        long int fileSize = ftell(file);                    //  Returns the actual file size
        if(fileSize < MAX_SIZE){                            
            fprintf(file, "%s\n", RTOS->imu.RollPitchYaw);  //  Fills the file with data
        }
        else{
            printf("FILE IS FULL! CLEANING 12 HOURS OF DATA!\n");              
            fseek(file, MAX_SIZE / 2, SEEK_SET);            //  Places the pointer in the middle of the file
            fread(temp, sizeof(char), MAX_SIZE / 2, file);  //  Reads the twelve most recent hours of data into "temp"
            fclose(file);
            
            file = fopen(STORAGE_PATH, "w");                //  Re-opens the file in write mode cleaning its content
            if (file == NULL) {
                perror("Error opening file.\n");
                return NULL;
            }
            fwrite(temp, sizeof(char), MAX_SIZE / 2, file); //  Writes in the empty file the stored recent data 
        }
        usleep(5000); 
    }
    fclose(file);
    return NULL;
}

void *transmitData(void *arg){
    IMUData *RTOS = (IMUData *)arg;
    ASM330LHB_uartInit(&RTOS->imu);
    RTOS->mode = REPLAY;

    FILE *file = fopen("/IMU/BlackBox.txt", "r");              
    if(file == NULL){
        perror("Error opening file.\n");
        return NULL;
    }
    else if(RTOS->mode == REPLAY){
        fseek(file, 0, SEEK_END);                           //  Places the file pointer in the end to check the file size
        long int fileSize = ftell(file);                    //  Returns the actual file size
        int replayTime = (fileSize * 0.00518) / 66;         //  5.18ms -> 66 bytes, So filesize bytes -> replayTime(seconds)
        printf(" Replay Time: %dmin and %dsec\n", (replayTime/60), (replayTime%60));        
        rewind(file); 
    }
    char line[21];
    char buffer[68] = {0};
    int counter = 0;              

    while (1)
    {
        switch (RTOS->mode)
        {
            case REAL_TIME:     //Transmits the current IMU data in real-time
                if(ASM330LHB_uartTransmit(&RTOS->imu, RTOS->imu.RollPitchYaw) != 0){
                    ASM330LHB_error("Error UART Transmit", RTOS->imu.uart_fd);
                    return NULL;
                }
            break;
            case REPLAY:        //Transmits the data stored in the raspberry
                while (fgets(line, sizeof(line), file) != NULL) 
                {
                    strcat(buffer, line);
                    counter++;
                    if (counter == 3) {
                        ASM330LHB_uartTransmit(&RTOS->imu, buffer);
                        memset(buffer, 0, sizeof(buffer));
                        counter = 0;
                    }
                }
                if (feof(file)) {
                    printf("RESTARTING REPLAY...\n");   
                    rewind(file);
                }
         
            break;
        }    
        usleep(SAMPLE_TIME * 1e6); 
    }
    fclose(file);
    return NULL;
}


int main() {
    // Inicialize a estrutura para armazenar os dados do IMU
    IMUData RTOS;
    pthread_mutex_init(&(RTOS.mutex), NULL);

    // Crie as threads
    pthread_t threads[NUM_THREADS];
    pthread_create(&(threads[0]), NULL, readIMU, (void *)&RTOS);
    pthread_create(&(threads[1]), NULL, transmitData, (void *)&RTOS);
    pthread_create(&(threads[2]), NULL, manageStorage, (void *)&RTOS);
    
    //pthread_create(&(threads[1]), NULL, processTask, (void *)&RTOS);

    // Aguarde o término das threads (isso nunca deve acontecer)
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], NULL);
    }

    // Libere o mutex
    pthread_mutex_destroy(&(RTOS.mutex));

    return 0;
}




/**
 * @brief Initializes UART and Configures its attributes
 * @param imu Pointer to the ASM330LHB structure containing UART's file descriptor
 * @return Returns 0 if the read operation was successful, otherwise returns -1.
*/
static __u8 ASM330LHB_uartInit(ASM330LHB *imu){

    imu->uart_fd = open(UART_DEV_PATH, O_RDWR);
    if (imu->uart_fd < 0)
        return ASM330LHB_error("Error opening UART file descriptor.", imu->uart_fd);

    struct termios options;
    tcgetattr(imu->uart_fd, &options);

    // Configurar velocidade de transmissão
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Configurar outros parâmetros da UART
    options.c_cflag |= (CLOCAL | CREAD);  // Habilitar leitura e escrita
    options.c_cflag &= ~PARENB;           // Desativar paridade
    options.c_cflag &= ~CSTOPB;           // 1 bit de parada
    options.c_cflag &= ~CSIZE;            // Limpar bits de tamanho de caractere
    options.c_cflag |= CS8;               // 8 bits de dados
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Modo crú
    options.c_oflag &= ~OPOST;            // Desativar processamento de saída
    options.c_cc[VMIN] = 1;               // Mínimo de bytes a serem lidos
    options.c_cc[VTIME] = 0;              // Timeout de leitura

    tcsetattr(imu->uart_fd, TCSANOW, &options);

    return 0;
}

/**
 * @brief Transmits data throught UART
 * @param imu Pointer to the ASM330LHB structure containing UART's file descriptor
 * @param message Message to be sent in UART
 * @return Returns 0 if the read operation was successful, otherwise returns -1.
*/
static __u8 ASM330LHB_uartTransmit(ASM330LHB *imu, char* message){
    int bytes_written = write(imu->uart_fd, message, strlen(message));
    if (bytes_written < 0) 
        return ASM330LHB_error("Error writing to UART", imu->uart_fd);
    return 0;
}


/*#include "ASM330LHB/ASM330LHB.h"

static ASM330LHB imu = {
        .i2c_fd       = 0,
        .uart_fd      = 0,
        .gyroRange    = 0,
        .accRange     = 0.0f, 
        .acc          = {0.0f, 0.0f, 0.0f},
        .gyro         = {0.0f, 0.0f, 0.0f},
        .temp_celsius = 0.0f,
        .Xant_Pitch   = 0.0f,
        .Xant_Roll    = 0.0f,
        .Xant_Yaw     = 0.0f,
        .Pant_Yaw     = 1.0f,
        .Pant_Pitch   = 1.0f,
        .Pant_Roll    = 1.0f
    };
    
int main()
{   
   // ASM330LHB *imu = getIMU();
    ASM330LHB_Init(&imu, GYRO_ODR_1670_HZ, ACC_ODR_1670_HZ, GYRO_RANGE_500_DPS, ACC_RANGE_2_G);
    while(1)
        ASM330LHB_Process(&imu);
    
    return 0;
}*/


// Função da tarefa de processamento
/*void *processTask(void *arg) {
    IMUData *data = (IMUData *)arg;

    while (1) {
        // Leia dados do sensor IMU e execute o filtro de Kalman
        pthread_mutex_lock(&(data->mutex));
        float roll = Kalman_Roll(data->imu.roll, &(data->imu));
        float pitch = Kalman_Pitch(data->imu.pitch, &(data->imu));
        float yaw = Kalman_Yaw(data->imu.yaw, &(data->imu));
        pthread_mutex_unlock(&(data->mutex));

        // Faça algo com os resultados do filtro de Kalman, se necessário

        // Aguarde um pequeno intervalo antes de processar o próximo conjunto de dados
        usleep(10000); // 10ms
    }

    return NULL;
}*/
