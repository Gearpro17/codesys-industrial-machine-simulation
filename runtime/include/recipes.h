#pragma once
#include <cmath>
#include <string>
#include <vector>

namespace plant {

    // Recipe table. The PLC owns the *active recipe*; this copy is shared by the
    // C++ SoftController, the tests and (later) the historian.
    struct Recipe {
        int         id;
        std::string name;
        double      expectedHeightMm;
        double      toleranceMm;
    };

    inline bool heightMatches(const Recipe& r, double measuredMm) {
        return std::abs(measuredMm - r.expectedHeightMm) <= r.toleranceMm;
    }

    inline const std::vector<Recipe>& defaultRecipes() {
        static const std::vector<Recipe> table{
            { 1, "Box M", 120.0, 2.0 },
            { 2, "Box S",  80.0, 2.0 },
            { 3, "Box L", 160.0, 2.0 },
        };
        return table;
    }

} // namespace plant