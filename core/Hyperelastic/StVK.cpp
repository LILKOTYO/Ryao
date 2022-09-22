#include <StVK.h>
#include <Matrix_Utils.h>
#include <iostream>

using namespace std;

namespace Ryao {
namespace VOLUME {

StVK::StVK(const REAL& mu, const REAL& lambda) :
    _mu(mu), _lambda(lambda) {}

std::string StVK::name() const {
    return "StVK";
}

}
}