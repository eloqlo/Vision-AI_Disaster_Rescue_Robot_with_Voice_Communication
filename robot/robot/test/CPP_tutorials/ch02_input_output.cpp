#include <iostream>

int main() {
    int i, j;
    std::cout << "Enter NUM1: ";
    std::cin >> i;
    std::cout << "Enter NUM2: ";
    std::cin >> j;

    std::cout << "NUM1 + NUM2 = " << i+j << std::endl;

    return 0;
}

/**
 * 네임스페이스: 소속을 지정해주는 역할.
 *     - std 라는 네임스페이스는 C++에서 흔히 사용하는 함수, 클래스, 객체, 유틸리티가 정의된 네임스페이스이다.
 *     - 네임스페이스는 내부 identifier(형식, 함수, 변수 등)에 범위를 부여해 여러 라이브러리를 포함할 때 이름이 충돌하는 것을 방지하려고 사용한다.
 */