#define RYAO_ARROW_DEBUG
#include "Logger.h"

namespace Ryao {
    int main() {
    #if defined(RYAO_DEBUG) || defined(RYAO_ARROW_DEBUG)
    Logger::Init();
    RYAO_INFO("The Start Point is:");
    // RYAO_INFO("The End Point is: {}", 2);
    // RYAO_INFO("The Dir is: {}", 3);
    // RYAO_INFO("The Head is: {}", 4);
    // RYAO_INFO("The Color is {}", 5);
    // RYAO_INFO("The Id is {}", 6);
    #endif
}
}