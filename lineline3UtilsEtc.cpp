#pragma once
#ifndef LINE3SOLUTIONUTILS_CPP
#define LINE3SOLUTIONUTILS_CPP

#include "libs/glm/glm.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "libs/glm/gtx/compatibility.hpp"
#include "libs/glm/gtx/intersect.hpp"
//#include <glm/gtx/intersect.inl>

namespace ll3s
{

	glm::vec3 constrainToSegment3(glm::vec3 p, glm::vec3 a, glm::vec3 b)
	{

		glm::vec3 ba = b - a;

		float t = glm::dot(p - a, ba) / glm::dot(ba, ba);

		float tc = glm::clamp(t, (float)0.0, (float)1.0); ///min(max(x, minVal), maxVal)

		return glm::lerp(a, b, tc);
	}

	glm::vec3 ProjectPointOnToPlane3(glm::vec3 p, glm::vec3 PlaneOrig, glm::vec3 PlaneNorm)
	{

		/////
		///proj point to plane
		//plane c with normal
		/////

		// Make a vector from your orig point to the point of interest:
		// v = point-orig (in each dimension);
		glm::vec3 v = p - PlaneOrig;
		//Take the dot product of that vector with the unit normal vector n
		// dist = vx*nx + vy*ny + vz*nz;

		float dist = v.x * PlaneNorm.x + v.y * PlaneNorm.y + v.z * PlaneNorm.z;

		// Multiply the unit normal vector by the distance, and subtract that vector from your point.
		// projected_point = point - dist*normal;

		glm::vec3 projected_point = p - dist * PlaneNorm;

		return projected_point;
	}

	float PerpendicularDistanceSegments3(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 &p0, glm::vec3 &p1)
	{ //glm::vec3 &p0, glm::vec3 &p1

		glm::vec3 n = d - c; //plane normal vector
		//n = glm::normalize(n); // need to normalize???

		glm::vec3 PlaneA = ProjectPointOnToPlane3(a, c, n);
		glm::vec3 PlaneB = ProjectPointOnToPlane3(b, c, n);

		glm::vec3 PlaneBA = PlaneB - PlaneA;

		float t = glm::dot(c - PlaneA, PlaneBA) / glm::dot(PlaneBA, PlaneBA);

		t = (PlaneA != PlaneB) ? t : 0.0f; // Zero's t if parallel

		glm::vec3 ABtoLineCD = glm::lerp(a, b, glm::clamp(t, (float)0.0, (float)1.0));

		glm::vec3 CDtoSegAB = constrainToSegment3(ABtoLineCD, c, d);
		glm::vec3 ABtoSegCD = constrainToSegment3(CDtoSegAB, a, b);

		p0 = CDtoSegAB;
		p1 = ABtoSegCD;

		float distance = glm::distance(CDtoSegAB, ABtoSegCD);
		return distance;
	}

}

#endif ///LINE3SOLUTIONUTILS_CPP
