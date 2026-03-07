#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

int main(void) {
    pid_t control_pid;

    printf("---------- [Main] Vision-AI Disaster Rescue Robot System Starting... -----------\n");

    // Control Process 생성
    control_pid = fork();
    if (control_pid == 0) {
        char *args[] = {"./control_process", NULL};
        execv(args[0], args);
        exit(1); // execv 실패 시 종료
    } else if (control_pid < 0) {
        perror("Control Process 생성 실패");
        exit(1);
    }

    // TODO: 추가 프로세스 생성 (Vision, Audio 등)
    
    
    /* Main Process Loop */
    while (1) {
        usleep(1000000);
    }

    printf("---------- [Main] Vision-AI Disaster Rescue Robot System Shutting Down... -----------\n");
    return 0;
}