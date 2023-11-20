# sentry_gimbal_public
RoboMaster2023 heu sentry robot gimbal.

I created this repository for the interview. 

This is a project for the embedded system of sentry robot gimbal. It was developed using CubeMX, VSCode and Ozone, compiled using arm-none-eabi-gcc, and operating on an STM32F4 microcontroller.

It implemented functions including gimbal control, autonomous recognition and tracking. But the code for visual recognition is not here. Relevant content has been collated and published in the journal as [Design of target recognition tracking and attack system based on Kalman filter](https://bzxb.cqut.edu.cn/paperinfo.aspx?paperid=10633). The main body of the code is in the Application folder. AimAssist.c is responsible for data processing, and Gimbal.c is responsible for gimbal control.

Because of the limited preparation time for the competition and the fact that I am not majoring in software engineering, there are many things about this code that are not programmable.
