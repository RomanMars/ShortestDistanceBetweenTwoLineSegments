#include <iostream>

#include <glm/glm.hpp>

#include "lineline3UtilsEtc.cpp"

#include "SegmentDistToSegmentSolution.cpp"

int main(int, char **)
{

    //────  ──────────────────────────────────────────────────────────────────────────────────
    /// Utils
    glm::vec3 cp(5.0f, 5.0f, 5.0f);
    glm::vec3 ca(-1.50f, 2.26f, 2.72f);
    glm::vec3 cb(10.0f, 10.26f, 10.72f);

    glm::vec3 vToSegment = ll3s::constrainToSegment3(cp, ca, cb);

    glm::vec3 a(5.0f, 5.0f, 5.0f);
    glm::vec3 b(-1.50f, 2.26f, 2.72f);
    glm::vec3 c(10.0f, 10.26f, 10.72f);
    glm::vec3 d(20.0f, 20.26f, 20.72f);

    glm::vec3 cp0;
    glm::vec3 cp1;

    float distance3 = ll3s::PerpendicularDistanceSegments3(a, b, c, d, cp0, cp1);

    /////
    //────  ──────────────────────────────────────────────────────────────────────────────────

    /// Solution

    /// Order of point matter -> glm::vec3 &InA1, const glm::vec3 &InB1, const glm::vec3 &InA2, const glm::vec3 &InB2
    // safe - vector 0 check
    csd::SegmentDistToSegmentSafe(a, c, b, d, cp0, cp1);
    // not save - > SegmentDistToSegment();

    float distancesolution = glm::distance(cp0, cp1);

    std::cout << "closest distance : " << distancesolution << std::endl;

    std::cout << "Point 1 :" << cp0.x << " " << cp0.y << " " << cp0.z << " " << std::endl;
    std::cout << "Point 2 :" << cp1.x << " " << cp1.y << " " << cp1.z << " " << std::endl;

    /////
    //────  ──────────────────────────────────────────────────────────────────────────────────

    std::cin.get();
}
