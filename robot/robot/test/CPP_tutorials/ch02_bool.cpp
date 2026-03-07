#include <iostream>
using namespace std;

int main() {
    bool value;
    value = true;
    cout << "value: " << value << endl;
    cout << "int bool: " << static_cast<int>(value) << endl;
    
    value = false;
    cout << "value: " << value << endl;
    cout << "int bool: " << static_cast<int>(value) << endl;

    return 0;
}