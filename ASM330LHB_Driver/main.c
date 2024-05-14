#include "main.h"

/**
 * @brief Thread that processes the IMU data
 */
static void *readIMU(void *arg){
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
 * @brief Thread that implements Circular Logging by replacing the oldest data with recent one
 * keeping the text file with the most recent 24 hours IMU data.
 */
static void *manageStorage(void *arg){
    IMUData *RTOS = (IMUData *)arg;

    char temp[MAX_SIZE / 2];                    // Temporary Buffer to save the twelve most recent hours of data
    FILE *file = fopen(STORAGE_PATH, "a+");     //  Opens in Read and Write mode
    if (file == NULL) {
        perror("Error opening file.\n");
        return NULL;
    }

    while(1)
    {
        if(RTOS->mode == REPLAY)                            //In replay mode there is no need to manage storage
            break;        
        fseek(file, 0, SEEK_END);                           //  Places the file pointer in the end to check the file size
        long int fileSize = ftell(file);                    //  Returns the actual file size
        if(fileSize < MAX_SIZE){                            
            fprintf(file, "%s\n", RTOS->imu.RollPitchYaw);  //  Fills the file with data
        }
        else{
            printf("THE FILE IS FULL! CLEANING 10 MINUTES OF DATA...\n");              
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

/**
 * @brief Thread that transmits through UART roll, pitch and yaw information
 * @test Operation mode: 
 *      REAL_TIME - gathers information from IMU and sends it through UART in real-time
 *      REPLAY    - reads storage file to recreate the movement of the previous real-time trial
 */
static void *transmitData(void *arg){
    IMUData *RTOS = (IMUData *)arg;
   
    char buffer[68] = {0};
    char line[21] = {0};
    int lineCounter = 0;              
    RTOS->mode = MODE;
    
    ASM330LHB_uartInit(&RTOS->imu);

    FILE *file = fopen(STORAGE_PATH, "r");              
    if(file == NULL)
    {
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
                    lineCounter++;
                    if (lineCounter == 3) {
                        ASM330LHB_uartTransmit(&RTOS->imu, buffer);
                        memset(buffer, 0, sizeof(buffer));
                        lineCounter = 0;
                    }
                }
                if (feof(file)) {   
                    printf("TRANSMITION COMPLETED - RESTARTING REPLAY...\n");   
                    rewind(file);       //  Points file to the begining
                }
            break;
        }    
        usleep(SAMPLE_TIME * 1e6); 
    }
    fclose(file);
    return NULL;
}

/**
 * @brief Main function which initializes the mutex and creates and starts the threads 
 */
int main() {

    IMUData RTOS;
    pthread_mutex_init(&(RTOS.mutex), NULL);

    pthread_t threads[NUM_THREADS];
    pthread_create(&(threads[0]), NULL, readIMU, (void *)&RTOS);
    pthread_create(&(threads[1]), NULL, transmitData, (void *)&RTOS);
    pthread_create(&(threads[2]), NULL, manageStorage, (void *)&RTOS);
    
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], NULL);
    }

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