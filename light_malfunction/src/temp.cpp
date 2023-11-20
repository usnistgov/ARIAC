#include<iostream>
#include <vector>


int main() {
    std::vector<int> adfg={123,3,4,5,6,7,8,9,0};
    int a = 5;
    int variable = 2345;
    std::string as = "name";
    printf("%s", as.c_str());
    std::cout << float(a)/2;
    for (auto &num333 : adfg) {
        std::cout << num333/2;
    }

    float intensity = 0.2;
    // auto color = ignition::math::Color(intensity, intensity, intensity);

    return 0;
}