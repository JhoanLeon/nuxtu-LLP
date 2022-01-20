# Nuxtu repository for the Low Level Programming selection process

This implementation is divided in two main tasks: Read ADC channels for sensors data and Manage I2C protocol to communicate with the master.

**ADC:**
The first task is done using DMA peripheral of the microcontroller. This decision was made mainly due to the advantages of working with this device and its easy handling in STM32 microcontrollers with HAL libraries. The code has the capability to read channels of the specified ADC peripheral and then compute the real variables and voltages according to datasheets and descriptions provided in the statement. 

**I2C:**
This task is executed by means of interrupts and callback functions. The flag for address matching run the corresponding callback where is the code to read instructions and respond with the requested information. The programmed instructions are the 4 specified in the requirements in addition to the detection command. The initial solution use CLOCK_STRETCHING to ensure that the processor has time to correctly answer to the information received. However, it is optional and NO_CLOCK_STRETCHING mode can be used, but the timming issues related to respond time and the HAL management of the I2C protocol can cause OVERRUN error.

**Note 1:** This code was done using the STM32CubeIDE 1.4.0 to generate basic configuration templates and debug the code.

**Note 2:** This implementation use HAL libraries to do all tasks. This because it favors the portability of the implementation between reference of STM32.

**Note 3:** FreeRTOS is not used due to there are not a lot of tasks to manage and they are realized with techniques that properly use the resources of the processor. For this implementation, the execution of tasks is done without much dificulty, so the use of an RTOS was not necessary. 
