
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

#include <pthread.h>
#include <stdio.h>
#include <unistd.h> // Para sleep()

#include "ASM330LHB/ASM330LHB.h"
//#include "kalman/kalman.h"
#define NUM_THREADS 2

// Estrutura para armazenar dados do IMU
typedef struct {
    ASM330LHB imu;
    pthread_mutex_t mutex;
} IMUData;

// Função da tarefa do IMU
void *imuTask(void *arg) {
    IMUData *data = (IMUData *)arg;
    
    ASM330LHB_Init(&(data->imu), GYRO_ODR_1670_HZ, ACC_ODR_1670_HZ, GYRO_RANGE_500_DPS, ACC_RANGE_2_G);

    while (1) {
        // Obtenha dados do sensor IMU
        pthread_mutex_lock(&(data->mutex));
        ASM330LHB_Process(&(data->imu));
        pthread_mutex_unlock(&(data->mutex));

        // Aguarde um pequeno intervalo antes de ler o próximo conjunto de dados
        usleep(5000); // 10ms
    }

    return NULL;
}

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

int main() {
    // Inicialize a estrutura para armazenar os dados do IMU
    IMUData data;
    pthread_mutex_init(&(data.mutex), NULL);

    // Crie as threads
    pthread_t threads[NUM_THREADS];
    pthread_create(&(threads[0]), NULL, imuTask, (void *)&data);
    //pthread_create(&(threads[1]), NULL, processTask, (void *)&data);

    // Aguarde o término das threads (isso nunca deve acontecer)
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], NULL);
    }

    // Libere o mutex
    pthread_mutex_destroy(&(data.mutex));

    return 0;
}
