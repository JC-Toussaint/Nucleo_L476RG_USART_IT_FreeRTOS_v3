https://scienceprog.com/freertos-on-stm32/

https://community.st.com/s/question/0D50X00009XkXf1SAF/what-is-the-best-way-to-receive-uart-serial-data-at-115k-baud-in-a-freertos-based-app

A good rule of thumb is to use SIGNALS for ISRs and have the receiving and transmissions in separatetasks.
Here's an example:

    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
    {
     if(huart->Instance == USART3)
     {
     osSemaphoreRelease(host_tx_semHandle);
     }
    }
     
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
     if(huart->Instance == USART3 )
     {
    osSignalSet( host_rx_taskHandle, HOST_RX_ISR_FLAG );
     }
    }‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍

These are the handlers for the tasks. On the transmission, I'm using DMA, so when the UART is finished transmission the semaphore is released.
Here are the tasks:

void host_rx_process(void const * argument)
    {
    // Buffer to hold message
    static uint8_t rx_buffer[HOST_RX_BUFFER] = {0};
    static uint8_t index = 0;
    static bool in_frame = false;
    uint8_t tmp_char = 0;
    HAL_UART_Receive_IT( &huart3, rx_tmp_buffer, 1 );
    for(;;)
        {
        osSignalWait( HOST_RX_ISR_FLAG, osWaitForever );
        tmp_char = *( huart3.pRxBuffPtr - huart3.RxXferSize );
        // Do whatever needs to be done to the new char that just arrived
        //....
        // Be sure to set the interrupt for the next char
        if( HAL_UART_Receive_IT( &huart3, (huart3.pRxBuffPtr - huart3.RxXferSize), 1 ) != HAL_OK)
        {
        Error_Handler();
        }
        }
    }

What's important here is that I'm waiting for the signal to the rx process which is set by the ISR handler. 
Doing this saves the most amount of cycles on the ISR so the RTOS can manage the tasks effectively.