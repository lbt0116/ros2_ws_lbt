#ifndef ROBOT_SOFTWARE_GAITSCHEDULE_HPP
#define ROBOT_SOFTWARE_GAITSCHEDULE_HPP

#include <Eigen/Dense>
#include <array>
#include <stdexcept>
#include <algorithm>

namespace Galileo
{
struct GaitSchedule
{
    Eigen::Matrix<int, Eigen::Dynamic, 4> gaitSequence;
    double swingHeight;
    double stand_T;
    double swing_T;
    int gaitTag;
    double standHeight;
};

namespace GaitSchedules
{
const GaitSchedule STAND = {
    (Eigen::Matrix<int, 1, 4>() << 1, 1, 1, 1).finished(),
    0.0,  // swingHeight
    1.0,  // stand_T
    0.0,  // swing_T
    0,    // gaitTag
    0.55  // standHeight
};

const GaitSchedule TROT = {
    (Eigen::Matrix<int, 2, 4>() << 1, 0, 0, 1, 0, 1, 1, 0).finished(),
    0.1,  // swingHeight
    0.2,  // stand_T
    0.2,  // swing_T
    1,    // gaitTag
    0.55  // standHeight
};

const GaitSchedule BOUND = {
    (Eigen::Matrix<int, 2, 4>() << 0, 1, 0, 1, 1, 0, 1, 0).finished(),
    0.08,  // swingHeight
    0.25,  // stand_T
    0.25,  // swing_T
    2,     // gaitTag
    0.55   // standHeight (使用默认值)
};

const GaitSchedule PACE = {
    (Eigen::Matrix<int, 2, 4>() << 1, 1, 0, 0, 0, 0, 1, 1).finished(),
    0.08,  // swingHeight
    0.25,  // stand_T
    0.25,  // swing_T
    3,     // gaitTag
    0.55   // standHeight (使用默认值)
};

const GaitSchedule FLYTROT = {
    (Eigen::Matrix<int, 4, 4>() << 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0).finished(),
    0.08,  // swingHeight
    0.2,   // stand_T
    0.3,   // swing_T
    4,     // gaitTag
    0.65   // standHeight
};

const GaitSchedule STANDTROT = {
    (Eigen::Matrix<int, 4, 4>() << 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1).finished(),
    0.15,  // swingHeight
    0.3,   // stand_T
    0.25,  // swing_T
    5,     // gaitTag
    0.55   // standHeight
};

inline const GaitSchedule& getGaitByTag(int gaitTag) {
    static const std::array<int, 6> validTags = {0, 1, 2, 3, 4, 5};
    
    if (std::find(validTags.begin(), validTags.end(), gaitTag) == validTags.end()) {
        throw std::invalid_argument("Invalid gait tag: " + std::to_string(gaitTag));
    }
    
    switch (gaitTag) {
        case 0:
            return STAND;
        case 1:
            return TROT;
        case 2:
            return BOUND;
        case 3:
            return PACE;
        case 4:
            return FLYTROT;
        case 5:
            return STANDTROT;
        default:
            return STAND;  // 这行代码实际上永远不会执行，因为我们已经检查了无效的tag
    }
}

}  // namespace GaitSchedules

}  // namespace Galileo

#endif  // ROBOT_SOFTWARE_GAITSCHEDULE_HPP