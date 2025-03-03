# autoware_utils_math

## Overview

The **autoware_utils** library is a comprehensive toolkit designed to facilitate the development of autonomous driving applications.
This package provides essential utilities for mathematics.
It is extensively used in the Autoware project to handle common tasks such as value normalization and unit conversion.

## Design

- **`accumulator.hpp`**: A class for accumulating statistical data, supporting min, max, and mean calculations.
- **`constants.hpp`**: Defines commonly used mathematical constants like π and gravity.
- **`normalization.hpp`**: Functions for normalizing angles and degrees.
- **`range.hpp`**: Functions for generating sequences of numbers (arange, linspace).
- **`trigonometry.hpp`**: Optimized trigonometric functions for faster computation.
- **`unit_conversion.hpp`**: Functions for converting between different units (e.g., degrees to radians, km/h to m/s).

## Example Code Snippets

### Using Accumulator from accumulator.hpp

```cpp
#include <autoware_utils_math/accumulator.hpp>

int main()
{
  autoware_utils_math::Accumulator<double> acc;

  acc.add(1.0);
  acc.add(2.0);
  acc.add(3.0);

  std::cout << "Mean: " << acc.mean() << "\n";
  std::cout << "Min: " << acc.min() << "\n";
  std::cout << "Max: " << acc.max() << "\n";
  std::cout << "Count: " << acc.count() << "\n";

  return 0;
}
```
