rm ./Output/*
rm ./obj/*
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/main.o               main.c 
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/croutine.o           croutine.c 
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/event_groups.o       event_groups.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/list.o               list.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/queue.o              queue.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/stream_buffer.o      stream_buffer.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/tasks.o              tasks.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/timers.o             timers.c
#armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/mpu_wrappers.o       ./portable/Common/mpu_wrappers.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/port.o               ./portable/RVDS/ARM_CA9/port.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./include/ -I./Utility -I./portable/RVDS/ARM_CA9/ -o ./obj/heap_2.o               ./portable/MemMang/heap_2.c
armasm.894 --cpu=Cortex-A9  -o ./obj/portASM.o  	./portable/RVDS/ARM_CA9/portASM.s
armasm.894 --cpu=Cortex-A9  -o ./obj/startup.o  	./Utility/startup.s

armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./Utility/ -I./portable/RVDS/ARM_CA9/ -o ./obj/uart0.o              ./Utility/uart0.c
armcc.894 -W --cpu=Cortex-A9 -c  -I/opt/arm/4.1/standard-linux-pentium-rel/include/unix/ -I./Utility/ -I./portable/RVDS/ARM_CA9/ -o ./obj/utility.o            ./Utility/utility.c


armlink.894  --scatter ./Utility/CA7FreeRTOS.scat --entry=Start --libpath=/opt/arm/4.1/standard-linux-pentium-rel/lib/ --output obj/FreeRtos ./obj/*.o

fromelf.894 --m32 --output ./Output/FreeRtos.srec obj/FreeRtos
arm-linux-gnueabihf-objdump -D obj/FreeRtos > ./Output/FreeRtos.lst 


