# Field-Oriented Control (FOC) with STM32 B-G431B-ESC1 



## Field-Oriented Control (FOC)

![image-20240925200805714](.\README_sources\picture\FOC_structure.png)

## B-G431B-ESC1 

### Top view

<img src=".\README_sources\picture\top_view_B_G431B_ESC1.png" alt="image-20240925195401867" style="zoom:50%;" />

### Bottom view

<img src=".\README_sources\picture\bottom_view_B_G431B_ESC1.png" alt="image-20240925195613769" style="zoom:50%;" />

## Short Overview Of The Code

[foc.h](.\Core\Inc\foc.h): 			Parameters can be changed and basic settings can be made here.

[stm32g4xx_it.c](.\Core\Src\stm32g4xx_it.c):	TIM1_UP_TIM16_IRQHandler Triggers the code with 10kHz.

[task.c](.\Core\Src\task.c):			This is where the sequence and status of the program is controlled. 

[foc.c](.\Core\Src\foc.c):  			Current regulation & transformations are coordinated.

[foc_math.c](.\Core\Src\foc_math.c):		Functions for quick calculations . (LookUpTable,...).

[encoder.c](.\Core\Src\encoder.c):  		Determination of position and speed by reading out the encoder value.

[current_measurement.c](.\Core\Src\current_measurement.c): Calculates the current by reading the ADCs.

[svm.c](.\Core\Src\svm.c): 			Calculates the output for the ESC.











![image-20240927091331188](C:\Users\xboxg\Desktop\image-20240927091331188.png)

20 ms

![image-20240927092623143](C:\Users\xboxg\Desktop\image-20240927092623143.png)

![image-20240927095420926](C:\Users\xboxg\Desktop\image-20240927095420926.png)

![image-20240927100056288](C:\Users\xboxg\Desktop\image-20240927100056288.png)

![image-20240927101931674](C:\Users\xboxg\Desktop\foc_step.png)





Der Strom Regelkreis wird mit der tuning method des technischen Optimums erstellt, bei der die Übertragungsfunktion des geregelten Systems als die eines Tiefpassfilter erster Ordnung eingestellt wird.



The current control loop is created using the technical optimum tuning method, where the transfer function of the controlled system is set as that of a first order low pass filter.

The controller settings for the inner current control loop are set according to magnitude optimum and the controller parameters of the outer speed control loop are set according to the symmetrical optimum.

Der Geschwindigkeitsreglekreis wird über die Bandbreite des 



The controller settings for the inner current loop are set according to the magnitude optimum and the controller parameters for the outer speed loop are set according to the symmetry optimum.[]

[] Control of Electric Machine Drive Systems | Wiley Online Books. https://onlinelibrary.wiley.com/doi/book/10.1002/ 9780470876541, . – (Accessed on 08/06/2022)





![image-20241004102954214](C:\Users\xboxg\Desktop\image-20241004102954214.png)