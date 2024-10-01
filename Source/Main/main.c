#include "main.h"

/**
 * @brief Thread that processes the IMU data
 */
static void *readIMU(void *arg){
    IMUData *RTOS = (IMUData *)arg;
    
    ASM330LHB_Init(&(RTOS->imu), GYRO_ODR_208_HZ, ACC_ODR_208_HZ, GYRO_RANGE_125_DPS, ACC_RANGE_2_G);

    while(1){
       if(RTOS->mode == REAL_TIME){                 //In replay mode there is no need to process imu data
            pthread_mutex_lock(&(RTOS->mutex));
            ASM330LHB_Process(&(RTOS->imu));
            pthread_mutex_unlock(&(RTOS->mutex));

            usleep(5000);
       }
    }
    return NULL;                                    //Never gets here
}

/**
 * @brief Thread that detects veicule collisions
 */
static void *detectCrash(void *arg){
    IMUData *RTOS = (IMUData *)arg;
    ASM330LHB *imu = &RTOS->imu;
    
    FILE* file = fopen(STORAGE_PATH, "r");        
    if(file == NULL)
        ASM330LHB_error("Error opening file.",0);   

    while(1){
       if(RTOS->mode == REAL_TIME){     //In replay mode there is no need to process imu data
            //Threshold based collison detection
           /*printf("%f\n",imu->pitch);
            printf("%f\n",imu->roll);
            printf("%f\n",imu->acc_ms2[0]);
            printf("%f\n",imu->acc_ms2[1]);*/
            //Car rotation check
            if((imu->pitch > 80 || imu->pitch < -80) || (imu->roll > 60 || imu->roll < -60)   ||        //Car rotation check
            (imu->acc_ms2[0]  < -threshold_positive || imu->acc_ms2[0] > threshold_positive ||  //Car X Velocity change check
             imu->acc_ms2[1]  < -threshold_positive || imu->acc_ms2[1] > threshold_positive))
            { //The car is Upside-down or tilted side-ways(more than 90 degrees pitched or more then 60 degrees rolled)
                printf("A crash as been detected! Fetching Post-Crash data(30 seconds)\n");
                sleep(5);
                
                RTOS->mode = REPLAY;            //  Changes Transmition Mode
                //Calculate Replay Time
                fseek(file, 0, SEEK_END);                           //  Places the file pointer in the end to check the file size
                long int fileSize = ftell(file);                    //  Returns the actual file size
                int replayTime = (fileSize * 0.00518) / 66;         //  5.18ms -> 66 bytes, So filesize bytes -> replayTime(seconds)REPLAY
                if(replayTime < MIN_REPLAY_TIME)
                    ASM330LHB_error("Replay Time is to short.",0);
                else
                    printf("[MODE]: REPLAY -> Replay Time: %dmin and %dsec\n", (replayTime/60), (replayTime%60));        
                rewind(file); 
            }
            usleep(5000);
       }
    }
    return NULL;                        //Never gets here
}


/**
 * @brief Thread that implements Circular Logging by replacing the oldest data with recent one
 * keeping the text file with the most recent 15 min IMU data.
 */
static void *manageStorage(void *arg){
    IMUData *RTOS = (IMUData *)arg;
    ASM330LHB *imu = &RTOS->imu;
    
    char temp[MAX_SIZE / 2];                                    //  Temporary Buffer to save the 7.5 most recent min of data
    FILE *file = fopen(STORAGE_PATH, "a+");                     //  Opens in Read and Write mode
    if (file == NULL) 
        ASM330LHB_error("Error opening file.\n",0);

    while(1)
    {
        if(RTOS->mode == REAL_TIME){                            //  In replay mode there is no need to manage storage
            fseek(file, 0, SEEK_END);                           //  Places the file pointer in the end to check the file size
            long int fileSize = ftell(file);                    //  Returns the actual file size
        
            if(fileSize < MAX_SIZE)                             //  Keeps storing data in the text file                   
                fprintf(file,"Roll-> %.2f degrees\n\rPitch-> %.2f degrees\n\rYaw-> %.2f degrees\n\r", imu->roll, imu->pitch, imu->yaw);  
            else{
                printf("THE FILE IS FULL! CLEANING 7.5 MINUTES OF DATA...\n");              
                fseek(file, MAX_SIZE / 2, SEEK_SET);            //  Places the pointer in the middle of the file
                fread(temp, sizeof(char), MAX_SIZE / 2, file);  //  Reads the twelve most recent hours of data into "temp"
                fclose(file);
                
                file = fopen(STORAGE_PATH, "w");                //  Re-opens the file in write mode cleaning its content
                if (file == NULL)
                    ASM330LHB_error("Error opening file.\n",0);
                fwrite(temp, sizeof(char), MAX_SIZE / 2, file); //  Writes in the empty file the stored recent data 
            }
            usleep(5000); 
        }
    }
    //Never gets here
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
    ASM330LHB *imu = &RTOS->imu;
    
    char uartMessage[66];
    char buffer[68] = {0};
    char line[21] = {0};
    int lineCounter = 0;              
    RTOS->mode = MODE_INIT;

    ASM330LHB_uartInit(imu);
    FILE *file = fopen(STORAGE_PATH, "w");                //  When rasp is initialized the file is resete since we ware dealing with diferent car crashes
    if (file == NULL)
        ASM330LHB_error("Error opening file.\n",0);
    fclose(file);
    
    file = fopen(STORAGE_PATH, "r");      
    if(file == NULL)
        ASM330LHB_error("Error opening file.",0);     

    while (1)
    {
        switch (RTOS->mode)
        {
            case REAL_TIME:     //Transmits the current IMU data in real-time
                sprintf(uartMessage, "Roll-> %.2f degrees\n\rPitch-> %.2f degrees\n\rYaw-> %.2f degrees\n\r", imu->roll, imu->pitch, imu->yaw);
                if(ASM330LHB_uartTransmit(imu, uartMessage) != 0)
                    ASM330LHB_error("Error UART Transmit", imu->uart_fd);
            break;
            case REPLAY:        //Transmits the data stored in the raspberry
                while (fgets(line, sizeof(line), file) != NULL) 
                {   
                    //Concatenate roll pitch and yaw from the file and trasmit them to UART
                    lineCounter++;
                    strcat(buffer, line); 
                    if (lineCounter == 3) {
                        ASM330LHB_uartTransmit(imu, buffer);
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
    //Never gets here
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
    pthread_create(&(threads[3]), NULL, detectCrash, (void *)&RTOS);
    
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
        ASM330LHB_error("Error opening UART file descriptor.", imu->uart_fd);

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
        ASM330LHB_error("Error writing to UART", imu->uart_fd);
    return 0;
}
