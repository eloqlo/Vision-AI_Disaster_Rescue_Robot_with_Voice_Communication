#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

int main(void) {
    pid_t control_pid, vision_pid;

    printf("---------- [Main] Vision-AI Disaster Rescue Robot System Starting... -----------\n");

    // Control Process 생성
    control_pid = fork();
    if (control_pid == 0) {
        char *args[] = {"./control_process", NULL};
        execv(args[0], args);
        perror("control_process execv 실패");
        exit(1);
    } else if (control_pid < 0) {
        perror("Control Process 생성 실패");
        exit(1);
    }
    printf("[Main] control_process 시작 (pid=%d)\n", control_pid);

    // Vision Process 생성
    vision_pid = fork();
    if (vision_pid == 0) {
        char *args[] = {"./vision_process", NULL};
        execv(args[0], args);
        perror("vision_process execv 실패");
        exit(1);
    } else if (vision_pid < 0) {
        perror("Vision Process 생성 실패");
        // control_process는 이미 살아있으므로 kill 후 종료
        kill(control_pid, SIGTERM);
        exit(1);
    }
    printf("[Main] vision_process 시작 (pid=%d)\n", vision_pid);

    /* Main Process Loop: 자식 프로세스 종료 감지 */
    while (1) {
        int status;
        // WNOHANG: 종료된 자식 없으면 즉시 반환 (non-blocking)
        pid_t dead = waitpid(-1, &status, WNOHANG);

        if (dead == control_pid) {
            printf("[Main] control_process 비정상 종료 (status=%d), 재시작...\n", status);
            control_pid = fork();
            if (control_pid == 0) {
                char *args[] = {"./control_process", NULL};
                execv(args[0], args);
                exit(1);
            }
        } else if (dead == vision_pid) {
            printf("[Main] vision_process 비정상 종료 (status=%d), 재시작...\n", status);
            vision_pid = fork();
            if (vision_pid == 0) {
                char *args[] = {"./vision_process", NULL};
                execv(args[0], args);
                exit(1);
            }
        }

        usleep(1000000); // 1초 주기로 감시
    }

    printf("---------- [Main] Vision-AI Disaster Rescue Robot System Shutting Down... -----------\n");
    return 0;
}